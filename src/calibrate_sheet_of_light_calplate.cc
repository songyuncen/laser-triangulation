#include <vector>
#include <iostream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#include "laser_triangulation.h"

using namespace std;
using namespace cv;

namespace cvfs = cv::utils::fs;

const int kWaitMs = 100;

static void Action();

int main(int argc, char *argv[]) {
  Action();
}

static void ExtractAndBackProjectionLaserPoints(cv::Mat &image, int threshold,
                                                cv::Mat &intrinsic, cv::Mat &distortin, cv::Mat &rvec, cv::Mat &tvec,
                                                std::vector<cv::Point3f> &points);
static void EstimatePoseAndBackProjection(const std::vector<cv::Point2f> &img_pts, const std::vector<cv::Point3f> &obj_pts,
                                          cv::Mat &intrinsic, cv::Mat &distortion,
                                          std::vector<cv::Point3f> &pts);

void Action() {
  //--------------------------------------------------------------------------------------------------------
  // Parameters
  //--------------------------------------------------------------------------------------------------------
  int num_calib_images = 20;
  int num_scan_images = 290;
  int threshold = 80;
  const char *folder = "images";
  int step = 19;

  //--------------------------------------------------------------------------------------------------------
  // Part 1. Calibrate camera
  //--------------------------------------------------------------------------------------------------------
  std::vector<std::vector<cv::Point3f>> object_points;
  GenerateObjectPointsList(object_points, 0.00375, { 7, 7 }, num_calib_images);

  std::vector<std::vector<cv::Point2f>> image_points;
  cv::Size image_size;

  for (int i = 0; i < num_calib_images; ++i) {
    std::ostringstream num;
    num << std::setw(2) << std::setfill('0') << i + 1;
    cv::String file_name = cv::String("connection_rod_calib_") + num.str() + cv::String(".png");
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
    cv::imshow("Demo", frame);
    cv::waitKey(kWaitMs);

    image_size = image.size();
  }


  cv::Mat intrinsic;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs, tvecs;
  double rms = cv::calibrateCamera(object_points, image_points, image_size,
                                   intrinsic, distortion, rvecs, tvecs);
  std::cout << "Overall RMS re-projection error is " << rms << std::endl;

  //--------------------------------------------------------------------------------------------------------
  // Part 2: Calibrate the orientation of the light plane with respect to the world coordinate system
  //--------------------------------------------------------------------------------------------------------
  // Extract 3d plane points
  std::vector<cv::Point3f> points;
  cv::Mat image19 = cv::imread(cvfs::join(folder, "connection_rod_lightline_019.png"), cv::IMREAD_GRAYSCALE);
  ExtractAndBackProjectionLaserPoints(image19, threshold, intrinsic, distortion, rvecs[18], tvecs[18], points);
  cv::Mat image20 = cv::imread(cvfs::join(folder, "connection_rod_lightline_020.png"), cv::IMREAD_GRAYSCALE);
  ExtractAndBackProjectionLaserPoints(image20, threshold, intrinsic, distortion, rvecs[19], tvecs[19], points);
  WritePointCloudPly(points, "points.ply");

  // Fit 3d plane
  cv::Vec4f plane;
  Fit3dPlane(points, plane);
  cv::FileStorage laser("laser.yml", cv::FileStorage::WRITE);
  laser << "laser" << plane;

  //--------------------------------------------------------------------------------------------------------
  // Part 3: Calibration of the movement of the object between the acquisition of two successive profiles
  //--------------------------------------------------------------------------------------------------------
  cv::Mat im1 = cv::imread(cvfs::join(folder, "caltab_at_position_1.png"), cv::IMREAD_GRAYSCALE);
  std::vector<cv::Point2f> centers1;
  if (!cv::findCirclesGrid(im1, { 7, 7 }, centers1)) {
    std::cout << "Fail to find circle grid in " << "caltab_at_position_1.png" << std::endl;
    return;
  }
  std::vector<cv::Point3f> pts1;
  EstimatePoseAndBackProjection(centers1, object_points[0], intrinsic, distortion, pts1);

  cv::Mat im2 = cv::imread(cvfs::join(folder, "caltab_at_position_2.png"), cv::IMREAD_GRAYSCALE);
  std::vector<cv::Point2f> centers2;
  if (!cv::findCirclesGrid(im2, { 7, 7 }, centers2)) {
    std::cout << "Fail to find circle grid in " << "caltab_at_position_2.png" << std::endl;
    return;
  }
  std::vector<cv::Point3f> pts2;
  EstimatePoseAndBackProjection(centers2, object_points[0], intrinsic, distortion, pts2);

  cv::Point3f offset{ 0.0, 0.0, 0.0 };
  for (size_t i = 0; i < pts1.size(); ++i) {
    offset += (pts2[i] - pts1[i]);
  }

  offset /= (int)pts1.size();
  offset /= step;
  cv::Vec3f movement(offset.x, offset.y, offset.z);

  cv::FileStorage fs("movement.yml", cv::FileStorage::WRITE);
  fs << "movement" << movement;

  //--------------------------------------------------------------------------------------------------------
  // Part 4: Apply the calibration transforms to an already acquired disparity image.
  //--------------------------------------------------------------------------------------------------------
  std::vector<cv::Point3f> pts;
  Reconstruction(folder, "connection_rod_", ".png", 3, num_scan_images, threshold,
                 intrinsic, distortion, plane, movement, pts, true);
  WritePointCloudPly(pts, "scan.ply");
  system("scan.ply");
}

void ShowLinePoints(cv::Mat &image, std::vector<cv::Point> &pts) {
  cv::Mat frame;
  cv::cvtColor(image, frame, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < pts.size(); ++i) {
    cv::circle(frame, pts[i], 1, { 0, 0, 255 });
  }
  cv::imshow("Demo", frame);
  cv::waitKey(kWaitMs);
}

void ExtractAndBackProjectionLaserPoints(cv::Mat &image, int threshold,
                                         cv::Mat &intrinsic, cv::Mat &distortin, cv::Mat &rvec, cv::Mat &tvec,
                                         std::vector<cv::Point3f> &points) {
  std::vector<cv::Point> img_pts;
  ExtractLaserLine(image, threshold, img_pts);

  ShowLinePoints(image, img_pts);

  std::vector<cv::Point2f> img_pts_2f;
  cv::Mat(img_pts).copyTo(img_pts_2f);
  BackProjection(img_pts_2f, intrinsic, distortin, rvec, tvec, points);
}

void EstimatePoseAndBackProjection(const std::vector<cv::Point2f> &img_pts, const std::vector<cv::Point3f> &obj_pts,
                                   cv::Mat &intrinsic, cv::Mat &distortion,
                                   std::vector<cv::Point3f> &pts) {

  cv::Mat rvec, tvec;
  cv::solvePnP(obj_pts, img_pts, intrinsic, distortion, rvec, tvec);
  BackProjection(img_pts, intrinsic, distortion, rvec, tvec, pts);
}
