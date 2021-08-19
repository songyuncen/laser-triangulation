#include "laser_triangulation.h"

void Fit3dPlane(const std::vector<cv::Point3f> &points, cv::Vec4f &plane) {
  std::vector<cv::Mat> pts;
  cv::split(points, pts);
  cv::Mat coords;
  cv::vconcat(pts, coords);
  coords = coords.t();
  cv::Mat centroid;
  cv::reduce(coords, centroid, 0, cv::REDUCE_AVG, CV_32F);
  cv::Mat tmp;
  cv::Mat normalized = coords - cv::repeat(centroid, coords.rows, 1);

  cv::Mat w, u, vt;
  cv::SVDecomp(normalized, w, u, vt);

  plane[0] = vt.at<float>(2, 0);
  plane[1] = vt.at<float>(2, 1);
  plane[2] = vt.at<float>(2, 2);
  plane[3] = -(plane[0] * centroid.at<float>(0, 0) +
               plane[1] * centroid.at<float>(0, 1) +
               plane[2] * centroid.at<float>(0, 2));

  // write plane ply file
  double z0 = 0.0;
  double x0 = 0.0;
  double y0 = -plane[3] / plane[1];

  double z1, z2;
  z1 = z2 = centroid.at<float>(0, 2) * 1.2;

  cv::Mat minv, maxv;
  cv::reduce(coords, minv, 0, cv::REDUCE_MIN, CV_32F);
  cv::reduce(coords, maxv, 0, cv::REDUCE_MAX, CV_32F);
  cv::Mat range = maxv - minv;

  double x1, y1, x2, y2;
  if (range.at<float>(0, 0) > range.at<float>(0, 1)) {
    x1 = minv.at<float>(0, 0);
    y1 = -(plane[0] * x1 + plane[2] * z1 + plane[3]) / plane[1];
    x2 = maxv.at<float>(0, 0);
    y2 = -(plane[0] * x2 + plane[2] * z2 + plane[3]) / plane[1];
  } else {
    y1 = minv.at<float>(0, 0);
    x1 = -(plane[1] * y1 + plane[2] * z1 + plane[3]) / plane[0];
    y2 = maxv.at<float>(0, 0);
    x2 = -(plane[1] * y2 + plane[2] * z2 + plane[3]) / plane[0];
  }

  std::vector<cv::Point3f> laser;
  laser.push_back({ x0, y0, z0 });
  laser.push_back({ x1, y1, z1 });
  laser.push_back({ x2, y2, z2 });
  WritePlanePly(laser, "laser.ply");
}

