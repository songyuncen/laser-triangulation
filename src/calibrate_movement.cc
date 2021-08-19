#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <iostream>

#include "laser_triangulation.h"

namespace cvfs = cv::utils::fs;

const double kGrid = 0.00375;
const int kStep = 19;
const char *kImageNames[2] = {
  "caltab_at_position_1.png",
  "caltab_at_position_2.png"
};

static bool VerifyImageFolder(const char *folder);

static void EstimatePoseAndBackProjection(const std::vector<cv::Point2f> &img_pts,
                                          const std::vector<cv::Point3f> &obj_pts,
                                          cv::Mat &intrinsic, cv::Mat &distortion,
                                          std::vector<cv::Point3f> &pts);

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cout << "Usage: calibrate_movement folder camera.yml" << std::endl;
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

  std::vector<std::vector<cv::Point3f>> tmp;
  GenerateObjectPointsList(tmp, kGrid, { 7, 7 }, 1);
  std::vector<cv::Point3f> obj_pts = tmp[0];

  cv::Mat im1 = cv::imread(cvfs::join(argv[1], kImageNames[0]), cv::IMREAD_GRAYSCALE);
  std::vector<cv::Point2f> centers1;
  if (!cv::findCirclesGrid(im1, { 7, 7 }, centers1)) {
    std::cout << "Fail to find circle grid in " << kImageNames[0] << std::endl;
    return EXIT_FAILURE;
  }
  std::vector<cv::Point3f> pts1;
  EstimatePoseAndBackProjection(centers1, obj_pts, intrinsic, distortion, pts1);

  cv::Mat im2 = cv::imread(cvfs::join(argv[1], kImageNames[1]), cv::IMREAD_GRAYSCALE);
  std::vector<cv::Point2f> centers2;
  if (!cv::findCirclesGrid(im2, { 7, 7 }, centers2)) {
    std::cout << "Fail to find circle grid in " << kImageNames[1] << std::endl;
    return EXIT_FAILURE;
  }
  std::vector<cv::Point3f> pts2;
  EstimatePoseAndBackProjection(centers2, obj_pts, intrinsic, distortion, pts2);

  cv::Point3f offset{ 0.0, 0.0, 0.0 };
  for (size_t i = 0; i < pts1.size(); ++i) {
    offset += (pts2[i] - pts1[i]);
  }

  offset /= (int)pts1.size();
  offset /= kStep;

  cv::FileStorage fs("movement.yml", cv::FileStorage::WRITE);
  fs << "movement" << offset;
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

void EstimatePoseAndBackProjection(const std::vector<cv::Point2f> &img_pts, const std::vector<cv::Point3f> &obj_pts,
                                   cv::Mat &intrinsic, cv::Mat &distortion,
                                   std::vector<cv::Point3f> &pts) {

  cv::Mat rvec, tvec;
  cv::solvePnP(obj_pts, img_pts, intrinsic, distortion, rvec, tvec);
  BackProjection(img_pts, intrinsic, distortion, rvec, tvec, pts);
}
