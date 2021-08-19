#include "laser_triangulation.h"

// plane[0] * x + plane[1] * y + plane[2] * z + plane[3] = 0
static void GetObjectPlaneInCameraCoordinate(cv::Mat &rvec, cv::Mat &tvec, cv::Vec4d &plane);

void BackProjection(std::vector<cv::Point> &im_pts,
                    cv::Mat &intrinsic, cv::Mat &distortion, cv::Mat &rvec, cv::Mat &tvec,
                    std::vector<cv::Point3f> &obj_pts) {
  std::vector<cv::Point2f> src, dst;
  
  cv::Mat(im_pts).copyTo(src);
  cv::undistortPoints(src, dst, intrinsic, distortion);
  
  cv::Vec4d obj_plane;
  GetObjectPlaneInCameraCoordinate(rvec, tvec, obj_plane);

  // calculate intersection points between the obj_plane and all the 
  // 3d points (w * dst.x, w * dst.y, w)

  for (size_t i = 0; i < dst.size(); ++i) {
    double w = -obj_plane[3] / (obj_plane[0] * dst[i].x + obj_plane[1] * dst[i].y + obj_plane[2]);
    obj_pts.push_back(cv::Point3f(w * dst[i].x, w * dst[i].y, w));
  }
}

void GetObjectPlaneInCameraCoordinate(cv::Mat &rvec, cv::Mat &tvec, cv::Vec4d &plane) {
  // construct the [R | t] matrix for the plane
  cv::Mat rot, rigid;
  cv::Rodrigues(rvec, rot);
  cv::hconcat(rot, tvec, rigid);

  cv::Mat origin = cv::Mat::zeros({ 1,4 }, CV_64F);
  cv::Vec4d dd;
  origin.at<double>(3, 0) = 1.0;
  origin = rigid * origin;
  cv::Mat unit_z = cv::Mat::zeros({ 1,4 }, CV_64F);
  unit_z.at<double>(2, 0) = 1.0;
  unit_z.at<double>(3, 0) = 1.0;
  unit_z = rigid * unit_z;
  unit_z = unit_z - origin;

  plane[0] = unit_z.at<double>(0, 0);
  plane[1] = unit_z.at<double>(1, 0);
  plane[2] = unit_z.at<double>(2, 0);
  plane[3] = -(plane[0] * origin.at<double>(0, 0) +
               plane[1] * origin.at<double>(1, 0) +
               plane[2] * origin.at<double>(2, 0));
}

