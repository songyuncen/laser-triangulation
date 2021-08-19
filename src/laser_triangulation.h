#ifndef LASER_TRIANGULATION_H_
#define LASER_TRIANGULATION_H_
#include <opencv2/opencv.hpp>
#include <vector>

void ExtractLaserLine(cv::Mat &image, int threshold, std::vector<cv::Point> &points);
void BackProjection(std::vector<cv::Point> &im_pts,
                    cv::Mat &intrinsic, cv::Mat &distortion, cv::Mat &rvec, cv::Mat &tvec,
                    std::vector<cv::Point3f> &obj_pts);
void WritePointCloudPly(std::vector<cv::Point3f> &points, const cv::String &file);

#endif // LASER_TRIANGULATION_H_

