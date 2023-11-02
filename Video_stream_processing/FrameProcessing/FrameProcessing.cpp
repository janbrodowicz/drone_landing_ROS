#include "FrameProcessing.h"

using namespace frame;


FrameProcessing::FrameProcessing(cv::Mat image, int height, int width, int pSX, int pSY) : 
    m_image(image),
    m_height(height),
    m_width(width),
    m_pSX(pSX),
    m_pSY(pSY)
{

}

cv::Mat FrameProcessing::grey_conversion_blur(int size, double std_dev)
{
    cv::Mat grey = cv::Mat::zeros(m_height, m_width, CV_8UC1);
    cv::cvtColor(m_image, grey, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grey, grey, cv::Size(size, size), std_dev);
    m_grey = grey;

    return grey;
}

std::tuple<cv::Mat, unsigned int> FrameProcessing::global_thresholding(void)
{
    std::tuple<cv::Mat, unsigned int> ret;
    cv::Mat IB_G = cv::Mat::zeros(m_height, m_width, CV_8UC1);
    double minG, maxG;

    cv::minMaxLoc(m_grey, &minG, &maxG);

    unsigned int thrG = (maxG - minG) * 0.3 + minG;

    for (int j = 0; j < m_width; j++)
    {
        for (int i = 0; i < m_height; i++)
        {
            IB_G.at<uchar>(i, j) = 255 * (m_grey.at<uchar>(i, j) < thrG);
        }
    }

    ret = {IB_G, thrG};

    return ret;
}

cv::Mat FrameProcessing::local_thresholding(int cSize)
{
    cv::Mat IB_C = cv::Mat::zeros(m_height, m_width, CV_8UC1);

    for (int j = cSize; j < m_width - cSize; j++)
    {
        for (int i = cSize; i < m_height - cSize; i++)
        {
            cv::Mat patch = m_grey(cv::Range(i - cSize, i + cSize), cv::Range(j - cSize, j + cSize));
            unsigned int thrC = (cv::mean(cv::mean(patch).val[0])).val[0];
            IB_C.at<uchar>(i, j) = 255 * (m_grey.at<uchar>(i, j) < thrC);
        }
    }

    return IB_C;
}

std::tuple<cv::Mat, cv::Mat> FrameProcessing::thresholding_in_windows(void)
{
    std::tuple<cv::Mat, cv::Mat> ret;
    cv::Mat IB = cv::Mat::zeros(m_height, m_width, CV_8UC1);
    cv::Mat thresholds = cv::Mat::zeros(floor(m_height / m_pSX) + 1, floor(m_width / m_pSY) + 1, CV_8UC1);

    for (int jj = 0; jj < m_width; jj = jj + m_pSY)
    {
        for (int ii = 0; ii < m_height; ii = ii + m_pSX)
        {
            cv::Mat patch = m_grey(cv::Range(ii, std::min(ii + m_pSX - 1, m_height)), cv::Range(jj, std::min(jj + m_pSY - 1, m_width)));
            double minP, maxP;
            cv::minMaxLoc(patch, &minP, &maxP);
            unsigned int thrW = (maxP - minP) * 0.5 + minP;
            thresholds.at<uchar>(floor(ii / m_pSX), floor(jj / m_pSY)) = thrW;

            for (int j = jj; j < std::min(jj + m_pSY, m_width); j++)
            {
                for (int i = ii; i < std::min(ii + m_pSX, m_height); i++)
                {
                    IB.at<uchar>(i, j) = 255 * (patch.at<uchar>(i % 128, j % 128) < thrW);
                }
            }
        }
    }

    ret = {IB, thresholds};

    return ret;
}

cv::Mat FrameProcessing::interpolation(cv::Mat thresholds)
{
    cv::Mat IB_L = cv::Mat::zeros(m_height, m_width, CV_8UC1);
    int iT, jT;
    double dX1, dX2, dY1, dY2;
    int th11, th12, th21, th22;
    double th1, th2, th;

    for (int jj = 0; jj < m_width; jj++)
    {
        for (int ii = 0; ii < m_height; ii++)
        {
            // Corners of the image
            if (ii < m_pSX / 2 && jj < m_pSY / 2 || ii > (m_height - m_pSX / 2) && jj < m_pSY / 2 || ii < m_pSX / 2 && jj > (m_width - m_pSY / 2) || ii > (m_height - m_pSX / 2) && jj > (m_width - m_pSY / 2))
            {
                iT = floor(ii / m_pSX);
                jT = floor(jj / m_pSY);
                th11 = thresholds.at<uchar>(iT, jT);
                th12 = thresholds.at<uchar>(iT, jT);
                th21 = thresholds.at<uchar>(iT, jT);
                th22 = thresholds.at<uchar>(iT, jT);
            }

            // Horizontal borders of the image
            if (ii > m_pSX / 2 && ii <= (m_height - m_pSX / 2) && jj < m_pSY / 2 || ii > m_pSX / 2 && ii <= (m_height - m_pSX / 2) && jj > (m_width - m_pSY / 2))
            {
                iT = floor((ii - m_pSX / 2) / m_pSX);
                jT = floor((jj - m_pSY / 2) / m_pSY);
                th11 = thresholds.at<uchar>(iT, jT);
                th12 = thresholds.at<uchar>(iT + 1, jT);
                th21 = thresholds.at<uchar>(iT, jT);
                th22 = thresholds.at<uchar>(iT + 1, jT);
            }

            // Vertical borders of the image
            if (jj > m_pSY / 2 && jj <= (m_width - m_pSY / 2) && ii < m_pSX / 2 || jj > m_pSY / 2 && jj <= (m_width - m_pSY / 2) && ii > (m_height - m_pSX / 2))
            {
                iT = floor((ii - m_pSX / 2) / m_pSX);
                jT = floor((jj - m_pSY / 2) / m_pSY);
                th11 = thresholds.at<uchar>(iT, jT);
                th12 = thresholds.at<uchar>(iT, jT);
                th21 = thresholds.at<uchar>(iT, jT + 1);
                th22 = thresholds.at<uchar>(iT, jT + 1);
            }

            // Inside of image
            if (ii >= m_pSX / 2 && ii < (m_height - m_pSX / 2) && jj >= m_pSY / 2 && jj < (m_width - m_pSY / 2))
            {
                iT = floor((ii - m_pSX / 2) / m_pSX);
                jT = floor((jj - m_pSY / 2) / m_pSY);
                th11 = thresholds.at<uchar>(iT, jT);
                th12 = thresholds.at<uchar>(iT + 1, jT);
                th21 = thresholds.at<uchar>(iT, jT + 1);
                th22 = thresholds.at<uchar>(iT + 1, jT + 1);
            }

            dX1 = ii - m_pSX / 2 - (iT - 0) * m_pSX;
            dX2 = (iT + 1) * m_pSX - (ii - m_pSX / 2);
            dY1 = jj - m_pSY / 2 - (jT - 0) * m_pSY;
            dY2 = (jT + 1) * m_pSY - (jj - m_pSY / 2);
            th1 = th11 * (dX2 / m_pSX) + th12 * (dX1 / m_pSX);
            th2 = th21 * (dX2 / m_pSX) + th22 * (dX1 / m_pSX);
            th = th1 * (dY2 / m_pSY) + th2 * (dY1 / m_pSY);
            IB_L.at<uchar>(ii, jj) = 255 * (m_grey.at<uchar>(ii, jj) < th);
        }
    }

    return IB_L;
}

cv::Mat FrameProcessing::dilatation(cv::Mat img, int size)
{
    cv::Mat IB_D = cv::Mat::zeros(m_height, m_width, CV_8UC1);
    cv::dilate(img, IB_D, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size)));

    return IB_D;
}

cv::Mat FrameProcessing::median_filter(cv::Mat img, int size)
{
    cv::Mat IB_M = cv::Mat::zeros(m_height, m_width, CV_8UC1);
    cv::medianBlur(img, IB_M, size);

    return IB_M;
}

cv::Mat FrameProcessing::erosion(cv::Mat img, int size)
{
    cv::Mat IB_E = cv::Mat::zeros(m_height, m_width, CV_8UC1);
    cv::erode(img, IB_E, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size)));

    return IB_E;
}

cv::Mat FrameProcessing::use_uav_mask(cv::Mat img, cv::Mat mask)
{
    cv::Mat IB_mask = img;

    for (int jj = 0; jj < m_width; jj++)
    {
        for (int ii = 0; ii < m_height; ii++)
        {
            if (mask.at<uchar>(ii, jj) == 0)
                IB_mask.at<uchar>(ii, jj) = 0;
        }
    }

    return IB_mask;
}

std::tuple<cv::Mat, cv::Mat, cv::Mat> FrameProcessing::connected_component_labelling(cv::Mat img)
{
    std::tuple<cv::Mat, cv::Mat, cv::Mat> ret;
    cv::Mat IB_S, stats, cent;
    cv::connectedComponentsWithStats(img, IB_S, stats, cent, 8, CV_32S, cv::CCL_DEFAULT);
    cv::normalize(IB_S, IB_S, 0, 255, cv::NORM_MINMAX, CV_8UC1, cv::Mat());

    ret = {IB_S, stats, cent};

    return ret;
}

cv::Mat FrameProcessing::bin_to_3ch(cv::Mat img)
{
    cv::Mat IB_VIS = cv::Mat::zeros(m_height, m_width, CV_8UC3);
    cv::Mat channel[3];
    split(IB_VIS, channel);
    channel[0] = img;
    channel[1] = img;
    channel[2] = img;
    merge(channel, 3, IB_VIS);

    return IB_VIS;
}