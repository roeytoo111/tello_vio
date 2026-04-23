#ifndef VO_H
#define VO_H

#include "vo_utils.h"

#include <atomic>
#include <string>

using namespace cv;
using namespace std;

class FrameQueue;
class StateSocket;

class VisualOdometry
{
public:
  VisualOdometry() = default;

  /**
   * @brief Run the VO pipeline on live Tello frames.
   * @param frame_queue  Thread-safe queue fed by VideoSocket
   * @param state_socket StateSocket providing velocity/height telemetry
   * @param camera_config_path Path to camera_config.yaml
   * @param run_flag     Set to false to stop the loop
   * @return 0 on success, -1 on error
   */
  int run(FrameQueue& frame_queue, StateSocket& state_socket,
          const string& camera_config_path, atomic<bool>& run_flag);
};

#endif // VO_H
