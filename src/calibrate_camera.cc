#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <iostream>
#include <sstream>
#include <vector>

namespace cvfs = cv::utils::fs;

const int kImageCount = 20;
const char *kImagePrefix = "connection_rod_calib_";
const char *kImageSuffix = ".png";

static bool VerifyImageFolder(const char *folder);
static void CalibrateImage(const char *folder);

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "Usage: calibrate_camera image_folder_path" << std::endl;
    return EXIT_FAILURE;
  }

  if (!VerifyImageFolder(argv[1])) {
    return EXIT_FAILURE;
  }

  CalibrateImage(argv[1]);
}

bool VerifyImageFolder(const char *folder) {
  if (!cvfs::isDirectory(folder)) {
    std::cout << "Argument: " << folder << " is not a valid directory." << std::endl;
    return false;
  }

  cv::String image_file = cvfs::join(folder, "connection_rod_calib_10.png");
  if (!cvfs::exists(image_file)) {
    std::cout << "No image file found in " << folder << " folder." << std::endl;
    return false;
  }

  try {
    cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
    cv::Mat centers;
    if (!cv::findCirclesGrid(image, { 7, 7 }, centers)) {
      std::cout << "No board found." << std::endl;
      return false;
    }
  } catch (cv::Exception &e) {
    std::cout << e.what() << std::endl;
    return false;
  }

  return true;
}

void CalibrateImage(const char *folder) {
  std::vector<std::vector<cv::Point2f>> image_points;

  for (int i = 0; i < kImageCount; ++i) {
    std::ostringstream num;
    num << std::setw(2) << std::setfill('0') << i + 1;
    cv::String file_name = cv::String(kImagePrefix) + num.str() + cv::String(kImageSuffix);
    cv::Mat image = cv::imread(cvfs::join(folder, file_name), cv::IMREAD_GRAYSCALE);
    std::vector<cv::Point2f> centers;
    if (!cv::findCirclesGrid(image, { 7, 7 }, centers)) {
      std::cout << "Fail to find circle grid in " << file_name << std::endl;
      return;
    }
    image_points.push_back(centers);

    cv::Mat frame;
    cv::cvtColor(image, frame, cv::COLOR_GRAY2BGR);
    cv::drawChessboardCorners(frame, { 7, 7 }, centers, true);
    cv::putText(frame, num.str(), { 10, 15 }, cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0, 0, 255 });
    cv::imshow("Circle Grid", frame);
    cv::waitKey(0);
  }
}
