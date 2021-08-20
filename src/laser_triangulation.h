#ifndef LASER_TRIANGULATION_H_
#define LASER_TRIANGULATION_H_
#include <opencv2/opencv.hpp>
#include <vector>

void ExtractLaserLine(cv::Mat &image, int threshold, std::vector<cv::Point> &points);
void BackProjection(const std::vector<cv::Point2f> &im_pts,
                    cv::Mat &intrinsic, cv::Mat &distortion, cv::Mat &rvec, cv::Mat &tvec,
                    std::vector<cv::Point3f> &obj_pts);
void Fit3dPlane(const std::vector<cv::Point3f> &points, cv::Vec4f &plane);

void GenerateObjectPointsList(std::vector<std::vector<cv::Point3f>> &points, double gap, const cv::Size &size, int n);

void WritePointCloudPly(const std::vector<cv::Point3f> &points, const cv::String &file);
void WritePlanePly(const std::vector<cv::Point3f> &plane, const cv::String &file);


void Reconstruction(const char *folder, const char *prefx, const char *suffix, int num_width, int n, int threshold,
                    cv::Mat &intrinsic, cv::Mat &distortion, cv::Vec4f &laser, cv::Vec3f &movement,
                    std::vector<cv::Point3f> &pts,
                    bool show = false, int wait_ms = 40);
#endif // LASER_TRIANGULATION_H_

