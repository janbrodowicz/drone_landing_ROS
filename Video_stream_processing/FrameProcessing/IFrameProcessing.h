#include <tuple>
#include <opencv2/opencv.hpp>


namespace frame
{

class IFrameProcessing
{
    public:
        
        virtual cv::Mat grey_conversion(void) = 0;

        virtual cv::Mat gaussian_blur(int size, double std_dev) = 0;

        virtual cv::Mat global_tresholding(void) = 0;

        virtual cv::Mat local_tresholding(void) = 0;

        virtual std::tuple<cv::Mat, cv::Mat> tresholding_in_windows(void) = 0;

        virtual cv::Mat interpolation(cv::Mat tresholds) = 0;

        virtual cv::Mat dilatation(cv::Mat img, int size) = 0;

        virtual cv::Mat median_filter(cv::Mat img, int size) = 0;

        virtual cv::Mat erosion(cv::Mat img, int size) = 0;

        virtual cv::Mat use_uav_mask(cv::Mat img, cv::Mat mask) = 0;

        virtual std::tuple<cv::Mat, cv::Mat, cv::Mat> connected_component_labelling(cv::Mat img) = 0;

        virtual cv::Mat bin_to_3ch(cv::Mat img) = 0;

        virtual cv::Mat get_image(void) = 0;

        virtual cv::Mat get_grey_image(void) = 0;

        virtual cv::Mat get_gauss_image(void) = 0;

};


} // namespace frame