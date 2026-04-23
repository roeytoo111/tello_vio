#ifndef VIDEOSOCKET_HPP
#define VIDEOSOCKET_HPP

#include <atomic>

#include <libavutil/frame.h>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

#include "base_socket.hpp"
#include "h264decoder.hpp"
#include "frame_queue.hpp"

#ifdef RUN_SLAM
#include "openvslam_api.hpp"
#endif // RUN_SLAM

// Forward declaration
class LocalizationManager;

/**
* @class VideoSocket
* @brief Class that enables video streaming from the tello and creates and manages the SLAM object if SLAM is enabled
*/
class VideoSocket : public BaseSocket{
public:

  /**
  * @brief Constructor
  * @param [in] io_service io_service object used to handle all socket communication
  * @param [in] drone_ip ip address of drone
  * @param [in] drone_port port number on the drone
  * @param [in] local_port port on the local machine used to communicate with the drone port mentioned above
  * @param [in] run reference to a bool that is set to off when the Tello object destructor is called
  * @param [in] camera_config_file path to camera configuration file
  * @param [in] vocabulary_file path to vocabulary file
  * @param [in] load_map_db_path path and file name from which the map must be loaded
  * @param [in] save_map_db_path path and file name to which the map must be saved
  * @param [in] mask_img_path path to pattern mask input images
  * @param [in] load_map bool for whether the map should be loaded
  * @param [in] continue_mapping continue adding to the map even when a map has been loaded
  * @param [in] scale scale for SLAM
  * @return none
  */
  VideoSocket(
    asio::io_service& io_service,
    const std::string& drone_ip,
    const std::string& drone_port,
    const std::string& local_port,
    bool& run,
    const std::string camera_config_file,
    const std::string vocabulary_file,
    const std::string load_map_db_path,
    const std::string save_map_db_path,
    const std::string mask_img_path,
    bool load_map,
    bool continue_mapping,
    float scale,
    LocalizationManager* localization_manager = nullptr
  );

  /**
  * @brief Destructor
  * @return none
  */
  ~VideoSocket();

  /**
  * @brief take a snapshot of the next frame
  * @return void
  */
  void setSnapshot();

  /**
  * @brief Set the frame queue for VO consumption. When set, decoded frames
  *        are pushed to this queue and the built-in imshow display is skipped.
  */
  void setFrameQueue(FrameQueue* queue) { frame_queue_ = queue; }

private:

  void handleResponseFromDrone(const std::error_code& error, size_t r) override;
  void handleSendCommand(const std::error_code& error, size_t bytes_sent, std::string cmd) override;

  void decodeFrame();
  void takeSnapshot(cv::Mat& image);

  enum{ max_length_ =  2048 };
  enum{ max_length_large_ =  65536 };
  bool received_response_ = true;

  char data_[max_length_];
  char frame_buffer_[max_length_large_];

  size_t first_empty_index = 0;
  int frame_buffer_n_packets_ = 0;

  H264Decoder decoder_;
  ConverterRGB24 converter_;
#ifdef RECORD
  std::unique_ptr<cv::VideoWriter> video;
#endif

#ifdef RUN_SLAM
  std::unique_ptr<OpenVSLAM_API> api_;
#endif // RUN_SLAM
  bool& run_;
  std::atomic<bool> snap_ = false;
  
  // Pointer to LocalizationManager for VIO
  LocalizationManager* localization_manager_;

  // Frame queue for external consumers (VO). When non-null, frames are
  // pushed here and the built-in imshow display is skipped.
  FrameQueue* frame_queue_ = nullptr;
};

#endif // VIDEOSOCKET_HPP
