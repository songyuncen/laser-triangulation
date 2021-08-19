#include "laser_triangulation.h"
#include <fstream>

void WritePointCloudPly(std::vector<cv::Point3f> &points, const cv::String &file) {
  std::ofstream ply;
  ply.open(file);

  // header
  ply << "ply" << std::endl;
  ply << "format ascii 1.0" << std::endl;
  ply << "element vertex " << points.size() << std::endl;
  ply << "property float x" << std::endl;
  ply << "property float y" << std::endl;
  ply << "property float z" << std::endl;
  ply << "property uchar red" << std::endl;
  ply << "property uchar green" << std::endl;
  ply << "property uchar blue" << std::endl;
  ply << "end_header" << std::endl;

  ply << std::setprecision(4);
  for (size_t i = 0; i < points.size(); ++i) {
    ply << points[i].x << " " << points[i].y << " " << points[i].z << " 255 0 0" << std::endl;
  }

  ply.close();
}

