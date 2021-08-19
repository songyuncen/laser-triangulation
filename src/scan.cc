#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#include "laser_triangulation.h"

namespace cvfs = cv::utils::fs;

const char *kImagePrefix = "connection_rod_";
const char *kImageSuffix = ".png";
const int kImageCount = 290;
int kThreshold = 80;

static bool VerifyImageFolder(const char *folder);

int main(int argc, char *argv[]) {
  if (argc < 5) {
    std::cout << "Usage: scan folder camera.yml laser.yml movement.yml [threshold]" << std::endl;
    return EXIT_FAILURE;
  }

  if (argc == 6) {
    kThreshold = std::atoi(argv[5]);
  }

  if (!VerifyImageFolder(argv[1])) {
    return EXIT_FAILURE;
  }

  cv::Mat intrinsic, distortion;
  std::vector<cv::Mat> rvecs, tvecs;
  try {
    cv::FileStorage fs(argv[2], cv::FileStorage::READ);
    fs["intrinsic"] >> intrinsic;
    fs["distortion"] >> distortion;
    fs["rvecs"] >> rvecs;
    fs["tvecs"] >> tvecs;
  } catch (cv::Exception &e) {
    std::cout << "Fail to read calibration data: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  cv::FileStorage laser_file(argv[3], cv::FileStorage::READ);
  cv::Vec4f laser;
  laser_file["laser"] >> laser;

  cv::FileStorage movement_file(argv[4], cv::FileStorage::READ);
  cv::Vec3f movement;
  movement_file["movement"] >> movement;

  std::vector<cv::Point3f> pts;
  Reconstruction(argv[1], kImagePrefix, kImageSuffix, 3, kImageCount, kThreshold,
                 intrinsic, distortion, laser, movement, pts);
  WritePointCloudPly(pts, "scan.ply");
}

bool VerifyImageFolder(const char *folder) {
  if (!cvfs::isDirectory(folder)) {
    std::cout << "Argument: " << folder << " is not a valid directory." << std::endl;
    return false;
  }

  cv::String name = cv::String(kImagePrefix) + "001" + kImageSuffix;
  cv::String image_file = cvfs::join(folder, name);
  if (!cvfs::exists(image_file)) {
    std::cout << "No image file found in " << folder << " folder." << std::endl;
    return false;
  }

  cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    return false;
  }

  return true;
}

