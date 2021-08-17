#include "laser_triangulation.h"

const double kSigma = 1.0;
const int kMinInterval = 1;

static cv::Mat getCannyFilter(double sigma);

void ExtractLaserLine(cv::Mat &image, int threshold, std::vector<cv::Point> &points) {
  points.clear();
  cv::Mat eye = cv::Mat::eye(1, 1, CV_64FC1);

  // smooth with gaussian
  int size = 2 * std::ceil(3.0 * kSigma) + 1;
  cv::Mat gaussian = cv::getGaussianKernel(size, kSigma);
  cv::Mat smooth;
  cv::sepFilter2D(image, smooth, CV_64F, eye, gaussian);

  // get derivative with canny filter
  cv::Mat canny = getCannyFilter(kSigma);
  cv::Mat derivative;
  cv::sepFilter2D(smooth, derivative, CV_64F, eye, canny);

  // find laser lines in every column of the image.
  cv::Mat im = image.t();
  derivative = derivative.t();
  for (int i = 0; i < im.rows; ++i) {
    uchar *val = im.ptr<uchar>(i);
    double *der = derivative.ptr<double>(i);

    // Locate intervals above threshold
    std::vector<int> intervals;
    int start = -1, end = -1;
    for (int j = 1; j < im.cols; ++j) {
      if (val[j - 1] < threshold && val[j] >= threshold) {
        start = j;
      } else if (val[j - 1] >= threshold && val[j] < threshold) {
        end = j;
        if (end - start > kMinInterval) {
          intervals.push_back(start);
          intervals.push_back(end);
        }
      }
    }
    
    // Locate the zero derivative points
    for (size_t j = 0; j < intervals.size() / 2; ++j) {
      int start = intervals[j * 2];
      int end = intervals[j * 2 + 1];

      double cur_min = DBL_MAX;
      int min_index = -1;
      for (int cur = start; cur <= end; ++cur) {
        double v = std::abs(der[cur]);
        if (v < cur_min) {
          cur_min = v;
          min_index = cur;
        }
      }

      points.push_back(cv::Point(i, min_index));
    }
  }
}


cv::Mat getCannyFilter(double sigma) {
  int r = std::ceil(sigma * 4.0);
  int size = 2 * r + 1;
  cv::Mat f = cv::Mat::zeros(cv::Size(1, size), CV_64F);
  double a = -0.5 / (sigma * sigma);
  double sum = 0.0;
  f.at<double>(r, 0) = 0.0;

  for (int x = 1; x <= r; ++x) {
    double v = -x * std::exp(x * x * a);
    f.at<double>(r + x, 0) = v;
    f.at<double>(r - x, 0) = -v;
    sum += v;
  }

  sum = std::abs(sum);
  return f / sum;
}

