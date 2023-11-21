#include "ObjectParameterAnalysis.h"

using namespace object;


ObjectParameterAnalysis::ObjectParameterAnalysis(cv::Mat img, cv::Mat stats, int circle_size, int square_size, int height, int width) :
    m_image(img),
    m_stats(stats),
    m_circle_size(circle_size),
    m_square_size(square_size),
    m_height(height),
    m_width(width)
{

};


std::vector<std::vector<double>> ObjectParameterAnalysis::bbox_cent(int i)
{
    std::vector<double> temp_bbox;
    std::vector<double> temp_cent;
    std::vector<std::vector<double>> ret;

    for (int j = 0; j < 4; j++)
        temp_bbox.push_back(m_stats.at<int>(i, j));
    for (int j = 0; j < 2; j++)
        temp_cent.push_back(m_stats.at<int>(i, j) + m_stats.at<int>(i, j + 2) / 2);

    ret = {temp_bbox, temp_cent};

    return ret;
}


std::vector<std::vector<std::vector<double>>> ObjectParameterAnalysis::all_det_obj(void)
{
    std::vector<std::vector<std::vector<double>>> ret;

    // Temporary vectors for centroids (cent) of circles (c), squares (s), rectangles (r)
    std::vector<std::vector<double>> centc;
    std::vector<std::vector<double>> cents;
    std::vector<std::vector<double>> centr;
    // Temporary vectors for bounding boxes (bb) of circles (c), squares (s), rectangles (r)
    std::vector<std::vector<double>> bbc;
    std::vector<std::vector<double>> bbs;
    std::vector<std::vector<double>> bbr;

    double bboxRatio;
    double bboxAreaRatio;
    double meanSquareSize;
    double bboxSize;

    for (int i = 1; i < m_stats.rows; i++)
    {
        bboxRatio = (double)std::max(m_stats.at<int>(i, 2), m_stats.at<int>(i, 3)) / (double)std::min(m_stats.at<int>(i, 2), m_stats.at<int>(i, 3));
        bboxAreaRatio = (double)m_stats.at<int>(i, 2) * (double)m_stats.at<int>(i, 3) / (double)m_stats.at<int>(i, 4);
        meanSquareSize = ((double)m_stats.at<int>(i, 2) + (double)m_stats.at<int>(i, 3)) / 2;
        bboxSize = (double)m_stats.at<int>(i, 2) * (double)m_stats.at<int>(i, 3);

        /** Detect big circle
         * @param bboxRatio         bounding box ratio is smaller than 1.4 (the shape of bbox is always similar to square)
         * @param bboxAreaRatio     bounding box area ratio is bigger than 3.0 (as it's rather a ring than a circle, many pixels inside bbox don't belong to the object)
         * @param meanSquareSize    mean square size of bounding box is bigger than 0.5 * m_circle_size (the average of m_height and m_width of bbox can't be too small at certain altitude)
         * @param meanSquareSize    mean square size of bounding box is smaller than 1.5 * m_circle_size (the average of m_height and m_width of bbox can't be too big at certain altitude)
         */
        /** Detect small circle
         * @param bboxRatio             bounding box ratio is smaller than 1.4 (the shape of bbox is always similar to square)
         * @param m_circle_size           circle size is bigger than 150 (the altitude is lower than a certain threshold)
         * @param m_circle_size           circle size is smaller than 500 (the altitude is higher than a certain threshold)
         * @param bboxAreaRatio         bounding box area ratio is bigger than 1.55 (no more than 65% of pixels inside bbox belong to the object)
         * @param bboxAreaRatio         bounding box area ratio is smaller than 2.0 (at least 50% of pixels inside bbox belong to the object)
         * @param meanSquareSize        mean square size of bounding box is smaller than 0.65 * m_square_size (the average of m_height and m_width of bbox can't be too big at certain altitude)
         * @param m_stats.at<int>(i, 4)   area of the object is smaller than 0.04 * m_circle_size * m_circle_size (the area of object can't be too big at certain altitude)
         * @param m_stats.at<int>(i, 4)   area of the object is bigger than 0.005 * m_circle_size * m_circle_size (the area of object can't be too small at certain altitude)
         or
         * @param bboxRatio             bounding box ratio is smaller than 1.4 (the shape of bbox is always similar to square)
         * @param m_circle_size           circle size is bigger than 500 (the altitude is lower than a certain threshold)
         * @param bboxAreaRatio         bounding box area ratio is bigger than 1.5 (no more than 67% of pixels inside bbox belong to the object)
         * @param bboxAreaRatio         bounding box area ratio is smaller than 2.0 (at least 50% of pixels inside bbox belong to the object)
         * @param m_stats.at<int>(i, 4)   area of the object is bigger than 0.01 * m_circle_size * m_circle_size (the area of object can't be too small at certain altitude)
         * @param m_stats.at<int>(i, 4)   area of the object is smaller than 0.04 * m_circle_size * m_circle_size (the area of object can't be too big at certain altitude)
         */
        if (bboxRatio < 1.4 && ((bboxAreaRatio > 3.0 && meanSquareSize > 0.5 * m_circle_size && meanSquareSize < 1.5 * m_circle_size) ||
                                (m_circle_size > 150 && m_circle_size < 500 && bboxAreaRatio > 1.55 && bboxAreaRatio < 2.0 && meanSquareSize < 0.65 * m_square_size &&
                                 m_stats.at<int>(i, 4) < 0.04 * m_circle_size * m_circle_size && m_stats.at<int>(i, 4) > 0.005 * m_circle_size * m_circle_size) ||
                                 (m_circle_size >= 500 && bboxAreaRatio > 1.5 && bboxAreaRatio < 2.0 && m_stats.at<int>(i, 4) > 0.01 * m_circle_size * m_circle_size && m_stats.at<int>(i, 4) < 0.04 * m_circle_size * m_circle_size)))
        {
            // Save the bounding box and centroid
            std::vector<std::vector<double>> b_c = bbox_cent(i);
            bbc.emplace_back(b_c[0]);
            centc.emplace_back(b_c[1]);
        }

        /** Detect square
         * @param bboxRatio             bounding box ratio is smaller than 1.4 (the shape of bbox is similar to square even if it's rotated)
         * @param bboxAreaRatio         bounding box area ratio is smaller than 2.25 (at least 44% of pixels inside bbox belong to the object)
         * @param bboxSize              bounding box size is bigger than 0.4 * m_square_size * m_square_size (the bbox can't be too small at certain altitude)
         * @param bboxSize              bounding box size is smaller than 2.0 * m_square_size * m_square_size (the bbox can't be too big at certain altitude)
         * @param m_stats.at<int>(i, 4)   area of the object is bigger than 0.3 * m_square_size * m_square_size (the area of object can't be too small at certain altitude)
         * @param m_stats.at<int>(i, 4)   area of the object is smaller than 1.8 * m_square_size * m_square_size (the area of object can't be too small at certain altitude)
         */
        else if (bboxRatio < 1.4 && bboxAreaRatio < 2.25 && bboxSize > 0.4 * m_square_size * m_square_size && bboxSize < 2.0 * m_square_size * m_square_size &&
                m_stats.at<int>(i, 4) > 0.3 * m_square_size * m_square_size && m_stats.at<int>(i, 4) < 1.8 * m_square_size * m_square_size)
        {
            // Save the bounding box and centroid
            std::vector<std::vector<double>> b_c = bbox_cent(i);
            bbs.push_back(b_c[0]);
            cents.push_back(b_c[1]);
        }

        /** Detect rectangle
         * @param bboxRatio             bounding box ratio equals at least 1.0 (the shape of bbox can be similar to square when it's rotated)
         * @param bboxRatio             bounding box ratio is smaller than 2.5 (the difference between m_height and m_width of bbox can't be too big)
         * @param bboxAreaRatio         bounding box area ratio is smaller than 2.3 (at least 43% of pixels inside bbox belong to the object)
         * @param bboxSize              bounding box size is bigger than 1.1 * m_square_size * m_square_size (the bbox can't be too small at certain altitude)
         * @param bboxSize              bounding box size is smaller than 10 * m_square_size * m_square_size (the bbox can't be too big at certain altitude)
         * @param m_stats.at<int>(i, 4)   area of the object is bigger than 0.5 * m_square_size * m_square_size (the area of object can't be too small at certain altitude)
         */
        else if (bboxRatio >= 1.0 && bboxRatio < 2.5 && bboxAreaRatio < 2.3 && bboxSize > 1.1 * m_square_size * m_square_size && bboxSize < 10 * m_square_size * m_square_size && m_stats.at<int>(i, 4) > 0.5 * m_square_size * m_square_size)
        {
            //Save the bounding box and centroid
            std::vector<std::vector<double>> b_c = bbox_cent(i);
            bbr.push_back(b_c[0]);
            centr.push_back(b_c[1]);
        }
    }

    ret = {centc, cents, centr, bbc, bbs, bbr};

    return ret;
}


std::tuple<cv::Mat, std::vector<double>, double> ObjectParameterAnalysis::draw_angle_cent(std::vector<std::vector<std::vector<double>>> detection_vec)
{
    std::tuple<cv::Mat, std::vector<double>, double> ret;
    std::vector<std::vector<double>> centc, cents, centr, bbc, bbs, bbr;
    double angle = -500;
    std::vector<double> centre;

    centc = detection_vec[0];
    cents = detection_vec[1];
    centr = detection_vec[2];
    bbc = detection_vec[3];
    bbs = detection_vec[4];
    bbr = detection_vec[5];

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
                    if (centc[i][0] > 0 && centc[i][1] > 0 && centc[i][0] < m_width && centc[i][1] < m_height &&
						cents[j][0] > 0 && cents[j][1] > 0 && cents[j][0] < m_width && cents[j][1] < m_height &&
						centr[k][0] > 0 && centr[k][1] > 0 && centr[k][0] < m_width && centr[k][1] < m_height)
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
                            cv::rectangle(m_image, rectc, cv::Scalar(0, 0, 255));
                            cv::Point pc;
                            pc.x = centc[i][0];
                            pc.y = centc[i][1];
                            if (pc.x > 0 && pc.y > 0)
                                cv::circle(m_image, pc, 2, cv::Scalar(0, 0, 255), 2);

                            // Draw the bounding box and the centroid of the square
                            cv::Rect rects(bbs[j][0], bbs[j][1], bbs[j][2], bbs[j][3]);
                            cv::rectangle(m_image, rects, cv::Scalar(0, 255, 0));
                            cv::Point ps;
                            ps.x = cents[j][0];
                            ps.y = cents[j][1];
                            if (ps.x > 0 && ps.y > 0)
                                cv::circle(m_image, ps, 2, cv::Scalar(0, 255, 0), 2);

                            // Draw the bounding box and the centroid of the rectangle
                            cv::Rect rectr(bbr[k][0], bbr[k][1], bbr[k][2], bbr[k][3]);
                            cv::rectangle(m_image, rectr, cv::Scalar(255, 0, 0));
                            cv::Point pr;
                            pr.x = centr[k][0];
                            pr.y = centr[k][1];
                            if (pr.x > 0 && pr.y > 0)
                                cv::circle(m_image, pr, 2, cv::Scalar(255, 0, 0), 2);

                            // If the distance is between 27,5% (based on the altitude) to 65% of the circle size, draw the line and calculate the position and orientation
                            if (sqrt(dist) < m_circle_size * 0.65 && sqrt(dist) > m_circle_size * 0.275)
                            {
                                cv::Point p1, p2;
                                p1.x = cents[j][0];
                                p1.y = cents[j][1];
                                p2.x = centr[k][0];
                                p2.y = centr[k][1];
                                cv::line(m_image, p1, p2, cv::Scalar(0, 255, 255), 2);

                                // Calculate the position as the average of square's and rectangle's centroid
                                cv::Point pos;
                                pos.x = (p1.x + p2.x) / 2;
                                pos.y = (p1.y + p2.y) / 2;
                                centre.emplace_back(pos.x);
                                centre.emplace_back(pos.y);
                                if (pos.x > 0 && pos.y > 0)
                                    cv::circle(m_image, pos, 2, cv::Scalar(0, 0, 255), 2);
                                // Calculate the angle, rotate it 90 degrees left (so now 0 degrees is up) and convert to degrees
                                angle = (atan2(cents[j][1] - centr[k][1], cents[j][0] - centr[k][0]) + M_PI / 2) * 180 / M_PI;
                                angle = int(angle);
                            }
                        }

                        // Draw the bounding box and the centroid of the small circle if it's between square and rectangle
                        if (m_circle_size > 150 && sqrt(dist) < m_circle_size * 0.65 &&
                            centc[i][0] >= std::min(cents[j][0], centr[k][0]) &&
                            centc[i][0] <= std::max(cents[j][0], centr[k][0]) &&
                            centc[i][1] >= std::min(cents[j][1], centr[k][1]) &&
                            centc[i][1] <= std::max(cents[j][1], centr[k][1]))
                        {
                            cv::Rect rectc(bbc[i][0], bbc[i][1], bbc[i][2], bbc[i][3]);
                            cv::rectangle(m_image, rectc, cv::Scalar(0, 0, 255));
                            cv::Point pc;
                            pc.x = centc[i][0];
                            pc.y = centc[i][1];
                            if (pc.x > 0 && pc.y > 0)
                                cv::circle(m_image, pc, 2, cv::Scalar(0, 0, 255), 2);
                        }
                    }
                }
            }
        }
    }

    // Save the centre and draw the bounding box and the centroid of the small circle if we're very close to the ground
    if (!bbc.empty() && m_circle_size > 500)
    {
        for (int i = 0; i < bbc.size(); i++)
        {
            cv::Rect rectc(bbc[i][0], bbc[i][1], bbc[i][2], bbc[i][3]);
            cv::rectangle(m_image, rectc, cv::Scalar(0, 0, 255));
            cv::Point pc;
            pc.x = centc[i][0];
            pc.y = centc[i][1];
            if (pc.x > 0 && pc.y > 0)
                cv::circle(m_image, pc, 2, cv::Scalar(0, 0, 255), 2);
            centre.emplace_back(pc.x);
            centre.emplace_back(pc.y);
        }
    }

    ret = {m_image, centre, angle};

    return ret;
}


std::vector<std::vector<double>> ObjectParameterAnalysis::calc_dist(std::vector<double> centre)
{
    std::vector<std::vector<double>> ret;
    std::vector<double> diff_to_centre(2);
    std::vector<double> distance_to_centre(2);
    
    if(!centre.empty())
    {
        diff_to_centre[0] = centre[0] - m_width / 2;
        diff_to_centre[1] = centre[1] - m_height / 2;
        distance_to_centre[0] = (25.0 / m_circle_size) * diff_to_centre[0];
        distance_to_centre[1] = (25.0 / m_circle_size) * diff_to_centre[1];
        diff_to_centre[0] = int(diff_to_centre[0]);
        diff_to_centre[1] = int(diff_to_centre[1]);
    }

    ret = {diff_to_centre, distance_to_centre};

    return ret;
}


cv::Mat ObjectParameterAnalysis::get_image(void) const
{
    return m_image;
}