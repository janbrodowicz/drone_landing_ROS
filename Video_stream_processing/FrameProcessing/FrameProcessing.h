#include <tuple>
#include <opencv2/opencv.hpp>


namespace frame
{

class FrameProcessing
{
    public:

        explicit FrameProcessing(cv::Mat image, int height, int width, int pSX, int pSY);

        cv::Mat grey_conversion_blur(int size, double std_dev);

        std::tuple<cv::Mat, unsigned int> global_thresholding(void);

        cv::Mat local_thresholding(int cSize);

        std::tuple<cv::Mat, cv::Mat> thresholding_in_windows(void);

        cv::Mat interpolation(cv::Mat thresholds);

        cv::Mat dilatation(cv::Mat img, int size);

        cv::Mat median_filter(cv::Mat img, int size);

        cv::Mat erosion(cv::Mat img, int size);

        cv::Mat use_uav_mask(cv::Mat img, cv::Mat mask);

        std::tuple<cv::Mat, cv::Mat, cv::Mat> connected_component_labelling(cv::Mat img);

        cv::Mat bin_to_3ch(cv::Mat img);

        cv::Mat get_image(void) const;

    private:

        cv::Mat m_image;

        cv::Mat m_grey;

        int m_height;

        int m_width;

        int m_pSX;

        int m_pSY;
};

} // namespace frame

