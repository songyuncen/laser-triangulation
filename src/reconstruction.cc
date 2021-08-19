#include "laser_triangulation.h"
#include <sstream>
#include <opencv2/core/utils/filesystem.hpp>

namespace cvfs = cv::utils::fs;

void Reconstruction(const char *folder, const char *prefix, const char *suffix, int num_width, int n, int threshold,
                    cv::Mat &intrinsic, cv::Mat &distortion, cv::Vec4f &laser, cv::Vec3f &movement,
                    std::vector<cv::Point3f> &pts) {
  pts.clear();

  for (int i = 0; i < n; ++i) {
    std::ostringstream num;
    num << std::setw(num_width) << std::setfill('0') << i + 1;
    cv::String file_name = cv::String(prefix) + num.str() + cv::String(suffix);
    cv::Mat im = cv::imread(cvfs::join(folder, file_name), cv::IMREAD_GRAYSCALE);
    std::vector<cv::Point> img_pts;
    ExtractLaserLine(im, threshold, img_pts);

    std::vector<cv::Point2f> src, dst;
    cv::Mat(img_pts).copyTo(src);
    cv::undistortPoints(src, dst, intrinsic, distortion);

    for (size_t j = 0; j < dst.size(); ++j) {
      double w = -laser[3] / (laser[0] * dst[j].x + laser[1] * dst[j].y + laser[2]);
      cv::Vec3f p(w * dst[j].x, w * dst[j].y, w);
      p -= (int)i * movement;
      pts.push_back(cv::Point3f(p[0], p[1], p[2]));
    }
  }
}

