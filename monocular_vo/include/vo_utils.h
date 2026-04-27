#ifndef VO_UTILS_H
#define VO_UTILS_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

using namespace cv;
using namespace std;

struct CameraCalibration {
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double k1 = 0, k2 = 0, k3 = 0, k4 = 0, k5 = 0;
  int cols = 960, rows = 720;
};

inline CameraCalibration parseCameraConfig(const string& config_path)
{
  CameraCalibration cal;
  ifstream file(config_path);
  if (!file.is_open()) {
    cerr << "Could not open camera config: " << config_path
         << " — using default Tello values" << endl;
    cal.fx = 1843.37401;  cal.fy = 1858.58782;
    cal.cx = 545.155626;  cal.cy = 445.740028;
    cal.k1 = 0.561939658; cal.k2 = -5.76385255;
    cal.k3 = 0.0356825441; cal.k4 = 0.0108696501;
    cal.k5 = 32.9089203;
    cal.cols = 960; cal.rows = 720;
    return cal;
  }

  string line;
  while (getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    size_t colon = line.find(':');
    if (colon == string::npos) continue;

    string key = line.substr(0, colon);
    string val = line.substr(colon + 1);
    val.erase(0, val.find_first_not_of(" \t\""));
    val.erase(val.find_last_not_of(" \t\"") + 1);

    try {
      if      (key == "Camera.fx")   cal.fx   = stod(val);
      else if (key == "Camera.fy")   cal.fy   = stod(val);
      else if (key == "Camera.cx")   cal.cx   = stod(val);
      else if (key == "Camera.cy")   cal.cy   = stod(val);
      else if (key == "Camera.k1")   cal.k1   = stod(val);
      else if (key == "Camera.k2")   cal.k2   = stod(val);
      else if (key == "Camera.k3")   cal.k3   = stod(val);
      else if (key == "Camera.k4")   cal.k4   = stod(val);
      else if (key == "Camera.k5")   cal.k5   = stod(val);
      else if (key == "Camera.cols") cal.cols  = stoi(val);
      else if (key == "Camera.rows") cal.rows  = stoi(val);
    } catch (...) {}
  }
  return cal;
}

inline void getCalibrationData(const string& config_path,
                                double& focal, Point2d& pp,
                                Mat& camera_matrix, Mat& dist_coeffs)
{
  CameraCalibration cal = parseCameraConfig(config_path);

  focal = (cal.fx + cal.fy) / 2.0;
  pp = Point2d(cal.cx, cal.cy);

  camera_matrix = (Mat_<double>(3, 3) <<
      cal.fx, 0,      cal.cx,
      0,      cal.fy, cal.cy,
      0,      0,      1);

  // OpenCV standard distortion: [k1, k2, p1, p2, k3]
  dist_coeffs = (Mat_<double>(5, 1) <<
      cal.k1, cal.k2, cal.k3, cal.k4, cal.k5);
}

inline void featureTracking(Mat img_1, Mat img_2,
                             vector<Point2f>& points1,
                             vector<Point2f>& points2,
                             vector<uchar>& status)
{
  if (points1.empty()) {
    points2.clear();
    status.clear();
    return;
  }

  vector<float> err;
  Size window_size = Size(21, 21);
  TermCriteria term_criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, window_size, 3, term_criteria, 0, 0.001);

  // Filter tracked points safely (avoid erase-while-iterating pitfalls).
  vector<Point2f> p1_f;
  vector<Point2f> p2_f;
  vector<uchar> st_f;
  p1_f.reserve(points1.size());
  p2_f.reserve(points2.size());
  st_f.reserve(status.size());

  const size_t n = std::min(points1.size(), points2.size());
  for (size_t i = 0; i < n && i < status.size(); i++) {
    const Point2f& pt2 = points2[i];
    if (status[i] == 0) continue;
    if (pt2.x < 0 || pt2.y < 0) continue;
    p1_f.push_back(points1[i]);
    p2_f.push_back(pt2);
    st_f.push_back(1);
  }

  points1.swap(p1_f);
  points2.swap(p2_f);
  status.swap(st_f);
}

inline void featureDetection(Mat img_1, vector<Point2f>& points1)
{
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool non_max_suppression = true;
  FAST(img_1, keypoints_1, fast_threshold, non_max_suppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}

#endif // VO_UTILS_H
