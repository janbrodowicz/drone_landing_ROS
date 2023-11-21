#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>
#include <optional>


namespace object
{

class ObjectParameterAnalysis
{
    public:

        ObjectParameterAnalysis(cv::Mat img, cv::Mat stats, int circle_size, int square_size, int height, int width);

        std::vector<std::vector<std::vector<double>>> all_det_obj(void);

        std::tuple<cv::Mat, std::vector<double>, double> draw_angle_cent(std::vector<std::vector<std::vector<double>>> detection_vec);

        std::vector<std::vector<double>> calc_dist(std::vector<double> centre);

        cv::Mat get_image(void) const;

    private:

        std::vector<std::vector<double>> bbox_cent(int i);

        cv::Mat m_image;

        cv::Mat m_stats;

        int m_circle_size;

        int m_square_size;

        int m_height;

        int m_width;
        
};


} // namespace object