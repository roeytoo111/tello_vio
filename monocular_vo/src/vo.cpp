#include "vo.h"
#include "frame_queue.hpp"
#include "state_socket.hpp"

#define MIN_NUM_FEAT 2000

static double estimateScale(StateSocket& state, double dt)
{
  double vx = state.getVgx() / 100.0;  // cm/s -> m/s
  double vy = state.getVgy() / 100.0;
  double vz = state.getVgz() / 100.0;
  double speed = sqrt(vx * vx + vy * vy + vz * vz);
  return speed * dt;
}

int VisualOdometry::run(FrameQueue& frame_queue, StateSocket& state_socket,
                        const string& camera_config_path, atomic<bool>& run_flag)
{
  // ---- calibration ----
  double focal;
  Point2d pp;
  Mat camera_matrix, dist_coeffs;
  getCalibrationData(camera_config_path, focal, pp, camera_matrix, dist_coeffs);

  // ---- undistortion maps (built from the first frame's resolution) ----
  Mat map1, map2;

  // ---- display helpers ----
  char text[200];
  int font_face  = FONT_HERSHEY_PLAIN;
  double font_scale = 1;
  int thickness   = 1;
  Point text_org(10, 50);

  // ---- wait for the first two frames ----
  Mat img_1_c, img_2_c;
  cout << "[VO] Waiting for first frame..." << endl;
  while (run_flag && !frame_queue.pop(img_1_c, 1000)) {}
  if (!run_flag) return 0;

  Size frame_size = img_1_c.size();
  Mat optimal_K = getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, frame_size, 0);
  initUndistortRectifyMap(camera_matrix, dist_coeffs, Mat(),
                          optimal_K, frame_size, CV_16SC2, map1, map2);

  cout << "[VO] Waiting for second frame..." << endl;
  while (run_flag && !frame_queue.pop(img_2_c, 1000)) {}
  if (!run_flag) return 0;

  // ---- undistort & convert to grey ----
  Mat img_1_u, img_2_u, img_1, img_2;
  remap(img_1_c, img_1_u, map1, map2, INTER_LINEAR);
  remap(img_2_c, img_2_u, map1, map2, INTER_LINEAR);
  cvtColor(img_1_u, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_u, img_2, COLOR_BGR2GRAY);

  // ---- initial feature detection & tracking ----
  vector<Point2f> points1, points2;
  featureDetection(img_1, points1);
  if (points1.empty()) {
    cout << "[VO] No features in first frame" << endl;
    return -1;
  }

  vector<uchar> status;
  featureTracking(img_1, img_2, points1, points2, status);
  if (points1.size() < 5 || points2.size() < 5) {
    cout << "[VO] Not enough features for initial pose" << endl;
    return -1;
  }

  // ---- initial pose ----
  Mat E, R, t, mask;
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  if (E.empty()) {
    cout << "[VO] Essential matrix is empty (init); skipping" << endl;
    return -1;
  }
  // OpenCV may return 3x(3N) when multiple solutions exist; recoverPose requires 3x3.
  if (E.rows == 3 && E.cols > 3) {
    E = E(Rect(0, 0, 3, 3)).clone();
  }
  if (E.rows != 3 || E.cols != 3) {
    cout << "[VO] Invalid essential matrix shape (init): " << E.rows << "x" << E.cols << endl;
    return -1;
  }
  recoverPose(E, points2, points1, R, t, focal, pp, mask);

  Mat R_f = R.clone();
  Mat t_f = t.clone();
  Mat prev_image = img_2;
  Mat curr_image;
  vector<Point2f> prev_features = points2;
  vector<Point2f> curr_features;

  namedWindow("Tello Camera", WINDOW_AUTOSIZE);
  namedWindow("Trajectory",   WINDOW_AUTOSIZE);
  Mat traj = Mat::zeros(600, 600, CV_8UC3);

  auto prev_time = chrono::steady_clock::now();
  int num_frame = 2;
  cout << "[VO] Running — press ESC in the Tello window to quit" << endl;

  while (run_flag)
  {
    Mat curr_image_c;
    if (!frame_queue.pop(curr_image_c, 500)) continue;

    auto curr_time = chrono::steady_clock::now();
    double dt = chrono::duration<double>(curr_time - prev_time).count();
    prev_time = curr_time;

    // Undistort
    Mat curr_undist;
    remap(curr_image_c, curr_undist, map1, map2, INTER_LINEAR);
    cvtColor(curr_undist, curr_image, COLOR_BGR2GRAY);

    // Track features
    vector<uchar> st;
    featureTracking(prev_image, curr_image, prev_features, curr_features, st);

    if (curr_features.size() < 5 || prev_features.size() < 5) {
      featureDetection(curr_image, prev_features);
      prev_image = curr_image.clone();
      num_frame++;
      continue;
    }

    E = findEssentialMat(curr_features, prev_features, focal, pp, RANSAC, 0.999, 1.0, mask);
    if (E.empty()) {
      // Degenerate configuration or too many outliers; re-detect and continue.
      featureDetection(curr_image, prev_features);
      prev_image = curr_image.clone();
      num_frame++;
      continue;
    }
    if (E.rows == 3 && E.cols > 3) {
      E = E(Rect(0, 0, 3, 3)).clone();
    }
    if (E.rows != 3 || E.cols != 3) {
      featureDetection(curr_image, prev_features);
      prev_image = curr_image.clone();
      num_frame++;
      continue;
    }
    recoverPose(E, curr_features, prev_features, R, t, focal, pp, mask);

    double scale = estimateScale(state_socket, dt);

    if ((scale > 0.01) &&
        (t.at<double>(2) > t.at<double>(0)) &&
        (t.at<double>(2) > t.at<double>(1)))
    {
      t_f = t_f + scale * (R_f * t);
      R_f = R * R_f;
    }

    // Re-detect when tracked features drop too low
    if ((int)prev_features.size() < MIN_NUM_FEAT)
    {
      featureDetection(prev_image, prev_features);
      featureTracking(prev_image, curr_image, prev_features, curr_features, st);
    }

    prev_image = curr_image.clone();
    prev_features = curr_features;

    // ---- trajectory visualisation (scaled to pixels) ----
    int draw_x = max(0, min(599, int(t_f.at<double>(0) * 100) + 300));
    int draw_y = max(0, min(599, int(-1 * t_f.at<double>(2) * 100) + 300));
    circle(traj, Point(draw_x, draw_y), 1, CV_RGB(255, 0, 0), 2);

    rectangle(traj, Point(10, 30), Point(590, 50), CV_RGB(0, 0, 0), FILLED);
    sprintf(text, "x=%.2fm y=%.2fm z=%.2fm  scale=%.4f",
            t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2), scale);
    putText(traj, text, text_org, font_face, font_scale, Scalar::all(255), thickness, 8);

    imshow("Tello Camera", curr_undist);
    imshow("Trajectory",   traj);
    waitKey(1);

    num_frame++;
  }

  cout << "[VO] Stopped after " << num_frame << " frames" << endl;
  return 0;
}
