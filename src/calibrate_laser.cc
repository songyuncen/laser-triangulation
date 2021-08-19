#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include "laser_triangulation.h"

namespace cvfs = cv::utils::fs;

const char *kImageNames[2] = {
  "connection_rod_lightline_019.png",
  "connection_rod_lightline_020.png"
};
const int kThreshold = 80;

static bool VerifyImageFolder(const char *folder);
static void ExtractAndBackProjectionLaserPoints(cv::Mat &image,
                                                cv::Mat &intrinsic, cv::Mat &distortin, cv::Mat &rvec, cv::Mat &tvec,
                                                std::vector<cv::Point3f> &points);

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cout << "Usage: calibrate_laser folder calibrate.yml" << std::endl;
    return EXIT_FAILURE;
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

  // 1. extract 3d plane points
  std::vector<cv::Point3f> points;
  cv::Mat image19 = cv::imread(cvfs::join(argv[1], kImageNames[0]), cv::IMREAD_GRAYSCALE);
  ExtractAndBackProjectionLaserPoints(image19, intrinsic, distortion, rvecs[18], tvecs[18], points);
  cv::Mat image20 = cv::imread(cvfs::join(argv[1], kImageNames[1]), cv::IMREAD_GRAYSCALE);
  ExtractAndBackProjectionLaserPoints(image20, intrinsic, distortion, rvecs[19], tvecs[19], points);
  
  WritePointCloudPly(points, "points.ply");


  // 2. fit 3d plane
  cv::Vec4f plane;
  Fit3dPlane(points, plane);
  cv::FileStorage laser("laser.yml", cv::FileStorage::WRITE);
  laser << "laser" << plane;
}

bool VerifyImageFolder(const char *folder) {
  if (!cvfs::isDirectory(folder)) {
    std::cout << "Argument: " << folder << " is not a valid directory." << std::endl;
    return false;
  }

  cv::String image_file = cvfs::join(folder, kImageNames[0]);
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

void ExtractAndBackProjectionLaserPoints(cv::Mat &image,
                                         cv::Mat &intrinsic, cv::Mat &distortin, cv::Mat &rvec, cv::Mat &tvec,
                                         std::vector<cv::Point3f> &points) {
  std::vector<cv::Point> img_pts;
  ExtractLaserLine(image, kThreshold, img_pts);
  BackProjection(img_pts, intrinsic, distortin, rvec, tvec, points);
}

