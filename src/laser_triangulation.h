#ifndef LASER_TRIANGULATION_H_
#define LASER_TRIANGULATION_H_
#include <opencv2/opencv.hpp>
#include <vector>

void ExtractLaserLine(cv::Mat &image, int threshold, std::vector<cv::Point> &points);

#endif // LASER_TRIANGULATION_H_

