#include "laser_triangulation.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char *argv[]) {
  if (argc != 2 && argc != 3) {
    std::cout << "Usage: extract image [threshold]" << std::endl;
    return EXIT_FAILURE;
  }

  int threshold = 80;
  if (argc == 3) {
    threshold = std::stoi(argv[2]);
  }

  cv::Mat image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    std::cout << "Can not open the image. " << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<cv::Point> points;
  ExtractLaserLine(image, threshold, points);
  cv::Mat frame;
  cv::cvtColor(image, frame, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < points.size(); ++i) {
    cv::circle(frame, points[i], 1, { 0, 0, 255 });
  }
  cv::imshow("Laser line", frame);
  cv::waitKey(0);
}

