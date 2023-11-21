#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <experimental/filesystem>

#include "AltitudeDataProcessing/AltitudeDataProcessing.h"
#include "FrameProcessing/FrameProcessing.h"
#include "ObjectParameterAnalysis/ObjectParameterAnalysis.h"

namespace fs = std::experimental::filesystem;


// -------------------------------------------------------------------------------------------------------------------
// CONSTANT PARAMETERS
// -------------------------------------------------------------------------------------------------------------------

#define M_PI 3.14159265358979323846

// The size of the input image
#define WIDTH 1280
#define HEIGHT 720

// The size of windows for thresholding
#define pSX 128
#define pSY 128

// Start frame and step
#define START_FRAME 0
#define STEP 1

// Mask visible parts of the drone
#define USE_MASK 0

// Average last 5 values from LIDAR or Pixhawk
#define AVERAGE 0

// Path to a directory containing a dataset
const fs::path PATH{"/home/jan/ROS_project/drone_landing_ROS/dataset/test_set1"};
const std::string PATH_HEIGHT = "/home/jan/ROS_project/drone_landing_ROS/dataset/test_set1_data.csv";
const std::string PATH_LOGS = "/home/jan/ROS_project/drone_landing_ROS/output/test_set1_logs_test.csv";

#if USE_MASK
const std::string PATH_MASK = "/home/jan/ROS_project/drone_landing_ROS/images/uav_mask.png";
#endif

int main()
{
    // Open directory with a dataset
    std::vector<std::string> listOfFiles;

    // // Load names of all files
    for (const auto & entry : fs::directory_iterator(PATH))
    {
        // std::cout << entry.path().filename() << std::endl;
        std::string tempName = std::string(entry.path().filename());
        listOfFiles.push_back(tempName);
    }
    
    std::sort(listOfFiles.begin(), listOfFiles.end());

    // Initialize output file
    std::ofstream outputFile(PATH_LOGS);
    if (outputFile.is_open())
        outputFile << "Number of objects" << ',' << " X centre [pix]" << ','
                   << " Y centre [pix]" << ',' << " X distance to centre [pix]" << ','
                   << " Y distance to centre [pix]" << ',' << " X distance to centre [cm]" << ','
                   << " Y distance to centre [cm]" << ',' << " Orientation angle [deg]" << std::endl;

    // Count the images and initialize tables with the size of a circle on consecutive frames
    int nimages = listOfFiles.end() - listOfFiles.begin() - 2;
    int table_circles[nimages];
    int table_squares[nimages];

    // -------------------------------------------------------------------------------------------------------------------
    // ALTITUDE DATA PROCESSING
    // -------------------------------------------------------------------------------------------------------------------

    // Open file with altitude data
    std::ifstream inputFile;
    inputFile.open(PATH_HEIGHT);
    if (inputFile.is_open())
    {
        std::string header;
        getline(inputFile, header);
        std::string altitude;
        double alt[nimages];
        std::string filename;

        // Use altitude data from LIDAR
        if (header.find("LIDAR") != std::string::npos)
        {
            for (int i = 0; i < nimages; i++)
            {
                getline(inputFile, filename, ',');
                inputFile >> altitude;
                double al = std::stod(altitude);
                altitude::LidarAltitude lidar(al);
                table_circles[i] = lidar.exp_circle_size();
            }
        }
        else if (header.find("Pixhawk") != std::string::npos)
        {
            for (int i = 0; i < nimages; i++)
            {
                getline(inputFile, filename, ',');
                inputFile >> altitude;
                double al = std::stod(altitude);
                altitude::PixhawkAltitude pixhawk(al);
                table_circles[i] = pixhawk.exp_circle_size();
            }
        }
        else
        {
            for (int i = 0; i < nimages; i++)
            {
                getline(inputFile, filename, ',');
                inputFile >> altitude;
                double al = std::stod(altitude);
                altitude::DefaultAltitude def(al);
                table_circles[i] = def.exp_circle_size();
            }
        }
    }
    inputFile.close();

    for (int i = 0; i < nimages; i++)
        table_squares[i] = int(table_circles[i] / 5);

    double centre[2] = {0, 0};
    double diff_to_centre[2] = {0, 0};
    double distance_to_centre[2] = {0, 0};
    double angle = 0;

    // -------------------------------------------------------------------------------------------------------------------
    // CONSECUTIVE FRAMES PROCESSING
    // -------------------------------------------------------------------------------------------------------------------

    for (std::vector<std::string>::iterator it = listOfFiles.begin() + 2 + START_FRAME; it < listOfFiles.end(); it = it + STEP)
    {
        // Load image
        cv::Mat image = cv::imread(std::string(PATH) + "/" + (*it));
        frame::FrameProcessing frame(image, HEIGHT, WIDTH, pSX, pSY);

        // Display debug info
        std::cout << "Processing file: " << (*it) << std::endl;

        // Conversion to grey and Gaussian blur
        cv::Mat grey = frame.grey_conversion_blur(5, 0.65);

        // Global thresholding
        std::tuple<cv::Mat, unsigned int> global_thr = frame.global_thresholding();

        // Local thresholding
        cv::Mat IB_C = frame.local_thresholding(7);

        // Thresholding in windows
        std::tuple<cv::Mat, cv::Mat> windows_thr = frame.thresholding_in_windows();

        // Interpolation of image thresholded in windows
        cv::Mat IB_L = frame.interpolation(std::get<1>(windows_thr));

        // Dilation 3x3
        cv::Mat IB_D = frame.dilatation(IB_L, 3);

        // Median filter 5x5
        cv::Mat IB_M = frame.median_filter(IB_D, 5);

        // Erosion 3x3
        cv::Mat IB_E = frame.erosion(IB_M, 3);

#if USE_MASK
        // Use the UAV mask
        cv::Mat mask = cv::imread(PATH_MASK);
        erosion = frame.use_uav_mask(erosion, mask)
#endif

        // Connected Component Labelling (CCL)
        std::tuple<cv::Mat, cv::Mat, cv::Mat> ccl = frame.connected_component_labelling(IB_E);

        // Convert binary image to 3-channel image
        cv::Mat IB_VIS = frame.bin_to_3ch(IB_E);

        // Extract stats from CCL
        cv::Mat stats = std::get<1>(ccl);

        int circle_size = table_circles[it - listOfFiles.begin() - 2];
        int square_size = table_squares[it - listOfFiles.begin() - 2];

        // Save to a file the number of objects
        if (outputFile.is_open())
            outputFile << stats.rows << ',';


        // -------------------------------------------------------------------------------------------------------------------
        // OBJECT PARAMETER ANALYSIS
        // -------------------------------------------------------------------------------------------------------------------

        object::ObjectParameterAnalysis object(IB_VIS, stats, circle_size, square_size, HEIGHT, WIDTH);

        // Iterate through all detected objects without background
        std::vector<std::vector<std::vector<double>>> detection_vec= object.all_det_obj();

        // Iterate through all detected circles, squares and rectangles
        std::tuple<cv::Mat, std::vector<double>, double> iter_res = object.draw_angle_cent(detection_vec);

        // Calculate the distance from the centre of the line to the centre of the image (in pixels and centimetres)
        cv::Mat image_det = std::get<0>(iter_res);
        std::vector<double> centre = std::get<1>(iter_res);
        double angle = std::get<2>(iter_res);
        std::vector<std::vector<double>> dist_vec = object.calc_dist(centre);

        // Save to a file the distance from centre in pixels and centimetres
        if (outputFile.is_open())
        {
            if(centre.empty())
                outputFile << "NULL" << ',' << "NULL" << ',' << "NULL" << ',' << "NULL" << ',' << "NULL" << ',' << "NULL" << ',';
            else
                outputFile << (int)centre[0] << ',' << (int)centre[1] << ',' << dist_vec[0][0] << ',' << dist_vec[0][1] << ',' << dist_vec[1][0] << ',' << dist_vec[1][1] << ',';
        }

        // Save to a file the orientation angle in degrees
        if (outputFile.is_open())
        {
            if(angle != -500)
                outputFile << angle;
            else
                outputFile << "NULL";
            outputFile << '\n';
        }

        cv::namedWindow("Detection result", cv::WINDOW_NORMAL);
        cv::imshow("Detection result", IB_VIS);
        std::string path_img = "/home/jan/ROS_project/drone_landing_ROS/output/out_test/detection_result_" + std::to_string(it - listOfFiles.begin() - 2)  + ".png";
        cv::imwrite(path_img, IB_VIS);
        std::cout <<" - done." << std::endl;
        cv::waitKey(1);
    }

    outputFile.close();
    return 0;
}
