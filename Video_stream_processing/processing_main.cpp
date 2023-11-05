#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <experimental/filesystem>

#include "AltitudeDataProcessing/AltitudeDataProcessing.h"
#include "FrameProcessing/FrameProcessing.h"

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

#if USE_MASK
    // Read a mask to remove UAV parts from the image
    cv::Mat mask = cv::imread(PATH_MASK);
    cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);
#endif

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

        // Temporary vectors for centroids (cent) of circles (c), squares (s), rectangles (r)
        std::vector<std::vector<double>> centc;
        std::vector<std::vector<double>> cents;
        std::vector<std::vector<double>> centr;
        // Temporary vectors for bounding boxes (bb) of circles (c), squares (s), rectangles (r)
        std::vector<std::vector<double>> bbc;
        std::vector<std::vector<double>> bbs;
        std::vector<std::vector<double>> bbr;

        // Save to a file the number of objects
        if (outputFile.is_open())
            outputFile << stats.rows << ',';

        // Parameters based on bounding box and area of the object
        double bboxRatio;
        double bboxAreaRatio;
        double meanSquareSize;
        double bboxSize;

        // -------------------------------------------------------------------------------------------------------------------
        // OBJECT PARAMETER ANALYSIS
        // -------------------------------------------------------------------------------------------------------------------

        // Iterate through all detected objects without background
        for (int i = 1; i < stats.rows; i++)
        {
            bboxRatio = (double)std::max(stats.at<int>(i, 2), stats.at<int>(i, 3)) / (double)std::min(stats.at<int>(i, 2), stats.at<int>(i, 3));
            bboxAreaRatio = (double)stats.at<int>(i, 2) * (double)stats.at<int>(i, 3) / (double)stats.at<int>(i, 4);
            meanSquareSize = ((double)stats.at<int>(i, 2) + (double)stats.at<int>(i, 3)) / 2;
            bboxSize = (double)stats.at<int>(i, 2) * (double)stats.at<int>(i, 3);

            /** Detect big circle
             * @param bboxRatio         bounding box ratio is smaller than 1.4 (the shape of bbox is always similar to square)
             * @param bboxAreaRatio     bounding box area ratio is bigger than 3.0 (as it's rather a ring than a circle, many pixels inside bbox don't belong to the object)
             * @param meanSquareSize    mean square size of bounding box is bigger than 0.5 * circle_size (the average of height and width of bbox can't be too small at certain altitude)
             * @param meanSquareSize    mean square size of bounding box is smaller than 1.5 * circle_size (the average of height and width of bbox can't be too big at certain altitude)
             */
            /** Detect small circle
             * @param bboxRatio             bounding box ratio is smaller than 1.4 (the shape of bbox is always similar to square)
             * @param circle_size           circle size is bigger than 150 (the altitude is lower than a certain threshold)
             * @param circle_size           circle size is smaller than 500 (the altitude is higher than a certain threshold)
             * @param bboxAreaRatio         bounding box area ratio is bigger than 1.55 (no more than 65% of pixels inside bbox belong to the object)
             * @param bboxAreaRatio         bounding box area ratio is smaller than 2.0 (at least 50% of pixels inside bbox belong to the object)
             * @param meanSquareSize        mean square size of bounding box is smaller than 0.65 * square_size (the average of height and width of bbox can't be too big at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is smaller than 0.04 * circle_size * circle_size (the area of object can't be too big at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is bigger than 0.005 * circle_size * circle_size (the area of object can't be too small at certain altitude)
             or
             * @param bboxRatio             bounding box ratio is smaller than 1.4 (the shape of bbox is always similar to square)
             * @param circle_size           circle size is bigger than 500 (the altitude is lower than a certain threshold)
             * @param bboxAreaRatio         bounding box area ratio is bigger than 1.5 (no more than 67% of pixels inside bbox belong to the object)
             * @param bboxAreaRatio         bounding box area ratio is smaller than 2.0 (at least 50% of pixels inside bbox belong to the object)
             * @param stats.at<int>(i, 4)   area of the object is bigger than 0.01 * circle_size * circle_size (the area of object can't be too small at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is smaller than 0.04 * circle_size * circle_size (the area of object can't be too big at certain altitude)
             */
            if (bboxRatio < 1.4 && ((bboxAreaRatio > 3.0 && meanSquareSize > 0.5 * circle_size && meanSquareSize < 1.5 * circle_size) ||
                                    (circle_size > 150 && circle_size < 500 && bboxAreaRatio > 1.55 && bboxAreaRatio < 2.0 && meanSquareSize < 0.65 * square_size &&
                                     stats.at<int>(i, 4) < 0.04 * circle_size * circle_size && stats.at<int>(i, 4) > 0.005 * circle_size * circle_size) ||
                                     (circle_size >= 500 && bboxAreaRatio > 1.5 && bboxAreaRatio < 2.0 && stats.at<int>(i, 4) > 0.01 * circle_size * circle_size && stats.at<int>(i, 4) < 0.04 * circle_size * circle_size)))
            {
                // Save the bounding box and centroid
                std::vector<double> temp_bbox;
                std::vector<double> temp_cent;
                for (int j = 0; j < 4; j++)
                    temp_bbox.push_back(stats.at<int>(i, j));
                for (int j = 0; j < 2; j++)
                    temp_cent.push_back(stats.at<int>(i, j) + stats.at<int>(i, j + 2) / 2);
                bbc.push_back(temp_bbox);
                centc.push_back(temp_cent);
            }

            /** Detect square
             * @param bboxRatio             bounding box ratio is smaller than 1.4 (the shape of bbox is similar to square even if it's rotated)
             * @param bboxAreaRatio         bounding box area ratio is smaller than 2.25 (at least 44% of pixels inside bbox belong to the object)
             * @param bboxSize              bounding box size is bigger than 0.4 * square_size * square_size (the bbox can't be too small at certain altitude)
             * @param bboxSize              bounding box size is smaller than 2.0 * square_size * square_size (the bbox can't be too big at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is bigger than 0.3 * square_size * square_size (the area of object can't be too small at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is smaller than 1.8 * square_size * square_size (the area of object can't be too small at certain altitude)
             */
            else if (bboxRatio < 1.4 && bboxAreaRatio < 2.25 && bboxSize > 0.4 * square_size * square_size && bboxSize < 2.0 * square_size * square_size &&
                    stats.at<int>(i, 4) > 0.3 * square_size * square_size && stats.at<int>(i, 4) < 1.8 * square_size * square_size)
            {
                // Save the bounding box and centroid
                std::vector<double> temp_bbox;
                std::vector<double> temp_cent;
                for (int j = 0; j < 4; j++)
                    temp_bbox.push_back(stats.at<int>(i, j));
                for (int j = 0; j < 2; j++)
                    temp_cent.push_back(stats.at<int>(i, j) + stats.at<int>(i, j + 2) / 2);
                bbs.push_back(temp_bbox);
                cents.push_back(temp_cent);
            }

            /** Detect rectangle
             * @param bboxRatio             bounding box ratio equals at least 1.0 (the shape of bbox can be similar to square when it's rotated)
             * @param bboxRatio             bounding box ratio is smaller than 2.5 (the difference between height and width of bbox can't be too big)
             * @param bboxAreaRatio         bounding box area ratio is smaller than 2.3 (at least 43% of pixels inside bbox belong to the object)
             * @param bboxSize              bounding box size is bigger than 1.1 * square_size * square_size (the bbox can't be too small at certain altitude)
             * @param bboxSize              bounding box size is smaller than 10 * square_size * square_size (the bbox can't be too big at certain altitude)
             * @param stats.at<int>(i, 4)   area of the object is bigger than 0.5 * square_size * square_size (the area of object can't be too small at certain altitude)
             */
            else if (bboxRatio >= 1.0 && bboxRatio < 2.5 && bboxAreaRatio < 2.3 && bboxSize > 1.1 * square_size * square_size && bboxSize < 10 * square_size * square_size && stats.at<int>(i, 4) > 0.5 * square_size * square_size)
            {
                //Save the bounding box and centroid
                std::vector<double> temp_bbox;
                std::vector<double> temp_cent;
                for (int j = 0; j < 4; j++)
                    temp_bbox.push_back(stats.at<int>(i, j));
                for (int j = 0; j < 2; j++)
                    temp_cent.push_back(stats.at<int>(i, j) + stats.at<int>(i, j + 2) / 2);
                bbr.push_back(temp_bbox);
                centr.push_back(temp_cent);
            }
        }

        // Iterate through all detected circles, squares and rectangles
        if ((!bbc.empty()) && (!cents.empty()) && (!centr.empty()))
        {
            std::cout << "All detected circles, squares, rectangles..." << std::endl;
            for (int i = 0; i < bbc.size(); i++)
            {
                for (int j = 0; j < cents.size(); j++)
                {
                    for (int k = 0; k < centr.size(); k++)
                    {
                        // Check if the centroids of the figures are inside the image
                        if (centc[i][0] > 0 && centc[i][1] > 0 && centc[i][0] < WIDTH && centc[i][1] < HEIGHT &&
							cents[j][0] > 0 && cents[j][1] > 0 && cents[j][0] < WIDTH && cents[j][1] < HEIGHT &&
							centr[k][0] > 0 && centr[k][1] > 0 && centr[k][0] < WIDTH && centr[k][1] < HEIGHT)
                        {
                            // Calculate the distance between the centroids of a square and a rectangle
                            double dist = (cents[j][0] - centr[k][0]) * (cents[j][0] - centr[k][0]) + (cents[j][1] - centr[k][1]) * (cents[j][1] - centr[k][1]);

                            // Check if there is a square and a rectangle inside a circle
                            if (bbc[i][0] < cents[j][0] && bbc[i][0] < centr[k][0] &&
                                bbc[i][1] < cents[j][1] && bbc[i][1] < centr[k][1] &&
                                bbc[i][0] + bbc[i][2] > cents[j][0] && bbc[i][0] + bbc[i][2] > centr[k][0] &&
                                bbc[i][1] + bbc[i][3] > cents[j][1] && bbc[i][1] + bbc[i][3] > centr[k][1])
                            {
                                // Draw the bounding box and the centroid of the circle
                                cv::Rect rectc(bbc[i][0], bbc[i][1], bbc[i][2], bbc[i][3]);
                                cv::rectangle(IB_VIS, rectc, cv::Scalar(0, 0, 255));
                                cv::Point pc;
                                pc.x = centc[i][0];
                                pc.y = centc[i][1];
                                if (pc.x > 0 && pc.y > 0)
                                    cv::circle(IB_VIS, pc, 2, cv::Scalar(0, 0, 255), 2);

                                // Draw the bounding box and the centroid of the square
                                cv::Rect rects(bbs[j][0], bbs[j][1], bbs[j][2], bbs[j][3]);
                                cv::rectangle(IB_VIS, rects, cv::Scalar(0, 255, 0));
                                cv::Point ps;
                                ps.x = cents[j][0];
                                ps.y = cents[j][1];
                                if (ps.x > 0 && ps.y > 0)
                                    cv::circle(IB_VIS, ps, 2, cv::Scalar(0, 255, 0), 2);

                                // Draw the bounding box and the centroid of the rectangle
                                cv::Rect rectr(bbr[k][0], bbr[k][1], bbr[k][2], bbr[k][3]);
                                cv::rectangle(IB_VIS, rectr, cv::Scalar(255, 0, 0));
                                cv::Point pr;
                                pr.x = centr[k][0];
                                pr.y = centr[k][1];
                                if (pr.x > 0 && pr.y > 0)
                                    cv::circle(IB_VIS, pr, 2, cv::Scalar(255, 0, 0), 2);

                                // If the distance is between 27,5% (based on the altitude) to 65% of the circle size, draw the line and calculate the position and orientation
                                if (sqrt(dist) < circle_size * 0.65 && sqrt(dist) > circle_size * 0.275)
                                {
                                    cv::Point p1, p2;
                                    p1.x = cents[j][0];
                                    p1.y = cents[j][1];
                                    p2.x = centr[k][0];
                                    p2.y = centr[k][1];
                                    cv::line(IB_VIS, p1, p2, cv::Scalar(0, 255, 255), 2);

                                    // Calculate the position as the average of square's and rectangle's centroid
                                    cv::Point pos;
                                    pos.x = (p1.x + p2.x) / 2;
                                    pos.y = (p1.y + p2.y) / 2;
                                    centre[0] = pos.x;
                                    centre[1] = pos.y;
                                    if (pos.x > 0 && pos.y > 0)
                                        cv::circle(IB_VIS, pos, 2, cv::Scalar(0, 0, 255), 2);
                                    // Calculate the angle, rotate it 90 degrees left (so now 0 degrees is up) and convert to degrees
                                    angle = (atan2(cents[j][1] - centr[k][1], cents[j][0] - centr[k][0]) + M_PI / 2) * 180 / M_PI;
                                    angle = int(angle);
                                }
                            }

                            // Draw the bounding box and the centroid of the small circle if it's between square and rectangle
                            if (circle_size > 150 && sqrt(dist) < circle_size * 0.65 &&
                                centc[i][0] >= std::min(cents[j][0], centr[k][0]) &&
                                centc[i][0] <= std::max(cents[j][0], centr[k][0]) &&
                                centc[i][1] >= std::min(cents[j][1], centr[k][1]) &&
                                centc[i][1] <= std::max(cents[j][1], centr[k][1]))
                            {
                                cv::Rect rectc(bbc[i][0], bbc[i][1], bbc[i][2], bbc[i][3]);
                                cv::rectangle(IB_VIS, rectc, cv::Scalar(0, 0, 255));
                                cv::Point pc;
                                pc.x = centc[i][0];
                                pc.y = centc[i][1];
                                if (pc.x > 0 && pc.y > 0)
                                    cv::circle(IB_VIS, pc, 2, cv::Scalar(0, 0, 255), 2);
                            }
                        }
                    }
                }
            }
        }

        // Save the centre and draw the bounding box and the centroid of the small circle if we're very close to the ground
        if (!bbc.empty() && circle_size > 500)
        {
            for (int i = 0; i < bbc.size(); i++)
            {
                cv::Rect rectc(bbc[i][0], bbc[i][1], bbc[i][2], bbc[i][3]);
                cv::rectangle(IB_VIS, rectc, cv::Scalar(0, 0, 255));
                cv::Point pc;
                pc.x = centc[i][0];
                pc.y = centc[i][1];
                if (pc.x > 0 && pc.y > 0)
                    cv::circle(IB_VIS, pc, 2, cv::Scalar(0, 0, 255), 2);
                centre[0] = pc.x;
                centre[1] = pc.y;
            }
        }

        // Calculate the distance from the centre of the line to the centre of the image (in pixels and centimetres)
        diff_to_centre[0] = centre[0] - WIDTH / 2;
        diff_to_centre[1] = centre[1] - HEIGHT / 2;
        distance_to_centre[0] = (25.0 / circle_size) * diff_to_centre[0];
        distance_to_centre[1] = (25.0 / circle_size) * diff_to_centre[1];
        diff_to_centre[0] = int(diff_to_centre[0]);
        diff_to_centre[1] = int(diff_to_centre[1]);

        // Save to a file the distance from centre in pixels and centimetres
        if (outputFile.is_open())
            outputFile << (int)centre[0] << ',' << (int)centre[1] << ',' << diff_to_centre[0] << ',' << diff_to_centre[1] << ',' << distance_to_centre[0] << ',' << distance_to_centre[1] << ',';

        // Save to a file the orientation angle in degrees
        if (outputFile.is_open())
        {
            outputFile << angle;
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
