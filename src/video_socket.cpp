#include <queue>
#include <mutex>
#include <iostream>
#include <chrono>

#include <libavutil/frame.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "video_socket.hpp"
#include "utils.hpp"

VideoSocket::VideoSocket(
  asio::io_service& io_service,
  const std::string& drone_ip,
  const std::string& drone_port,
  const std::string& local_port,
  bool& run,
  const std::string camera_config_file,
  const std::string vocabulary_file,
  const std::string load_map_db_path_,
  const std::string save_map_db_path_,
  const std::string mask_img_path_,
  bool load_map_,
  bool continue_mapping,
  float scale,
  LocalizationManager* localization_manager
):
  BaseSocket(io_service, drone_ip, drone_port, local_port),
  run_(run),
  localization_manager_(localization_manager)
{
  // cv::namedWindow("frame", CV_WINDOW_NORMAL);
  // cv::moveWindow("frame",960,0);
  // cv::resizeWindow("frame",920,500);

  cv::namedWindow("Pilot view");
  asio::ip::udp::resolver resolver(io_service_);
  asio::ip::udp::resolver::query query(asio::ip::udp::v4(), drone_ip_, drone_port_);
  asio::ip::udp::resolver::iterator iter = resolver.resolve(query);
  endpoint_ = *iter;

  socket_.async_receive_from(
    asio::buffer(data_, max_length_),
    endpoint_,
    [&](const std::error_code& error, size_t bytes_recvd)
    {return handleResponseFromDrone(error, bytes_recvd);});
    // [&](auto... args){return handleResponseFromDrone(args...);});

  io_thread = std::thread([&]{io_service_.run();
    utils_log::LogDebug() << "----------- Video socket io_service thread exits -----------";
  });
  io_thread.detach();

#ifdef RUN_SLAM
    api_ = std::make_unique<OpenVSLAM_API>(run_, camera_config_file, vocabulary_file, load_map_db_path_, save_map_db_path_, mask_img_path_, load_map_, continue_mapping, scale);
    api_->startMonoThread();
#endif

#ifdef RECORD
  std::string create_video_folder = "mkdir ../videos";
  system(create_video_folder.c_str());
  time_t rawtime;
  struct tm * timeinfo;
  char buffer [80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (buffer,80,"../videos/tello_video_%Y_%m_%d_%H_%M_%S.mp4",timeinfo);
  video = std::make_unique<cv::VideoWriter>(buffer, cv::VideoWriter::fourcc('m','p','4','v'), 30, cv::Size(960,720));
#endif

  std::string create_folder = "mkdir -p ../snapshots";
  system(create_folder.c_str());
}

void VideoSocket::handleResponseFromDrone(const std::error_code& error, size_t bytes_recvd)
{
  static int packet_count = 0;
  static auto last_log_time = std::chrono::steady_clock::now();
  
  if(first_empty_index == 0){
    first_empty_index = 0;
    frame_buffer_n_packets_ = 0;
  }

  if (error) {
    utils_log::LogWarn() << "VideoSocket error receiving packet: " << error.message();
  } else if (bytes_recvd > 0) {
    packet_count++;
    auto now = std::chrono::steady_clock::now();
    auto time_since_log = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();
    
    // Log packet reception every 5 seconds for debugging
    if (time_since_log >= 5) {
      utils_log::LogInfo() << "VideoSocket: Received " << packet_count << " packets in last 5 seconds";
      packet_count = 0;
      last_log_time = now;
    }
  }

  if (first_empty_index + bytes_recvd >= max_length_large_) {
    utils_log::LogInfo() << "Frame buffer overflow. Dropping frame";
    first_empty_index = 0;
    frame_buffer_n_packets_ = 0;
    // Don't return early: we must re-arm async_receive_from below.
    bytes_recvd = 0;
  }

  if (bytes_recvd > 0) {
    memcpy(frame_buffer_ + first_empty_index, data_, bytes_recvd );
    first_empty_index += bytes_recvd;
    frame_buffer_n_packets_++;
  }

  if (bytes_recvd < 1460 && bytes_recvd > 0) {
    decodeFrame();
    // decodeFrame() keeps any unconsumed bytes in the buffer, so we should not
    // blindly reset first_empty_index here.
    frame_buffer_n_packets_ = 0;
  }

  socket_.async_receive_from(
    asio::buffer(data_, max_length_),
    endpoint_,
    [&](const std::error_code& error, size_t bytes_recvd)
    {return handleResponseFromDrone(error, bytes_recvd);});
    // [&](auto... args){return handleResponseFromDrone(args...);});
}

void VideoSocket::decodeFrame()
{
  size_t next = 0;
  const size_t total = first_empty_index;
  try {
    while (next < total) {
      const ssize_t consumed =
        decoder_.parse(reinterpret_cast<const unsigned char*>(frame_buffer_) + next,
                       static_cast<ssize_t>(total - next));

      // If the parser doesn't consume input, break to avoid an infinite loop
      // on malformed/incomplete streams (e.g. missing SPS/PPS at startup).
      if (consumed <= 0) {
        utils_log::LogWarn() << "H264 parser consumed " << consumed
                             << " bytes. Waiting for more data.";
        break;
      }

      if (decoder_.is_frame_available()) {
        try {
          const AVFrame &frame = decoder_.decode_frame();
          std::vector<unsigned char> bgr24(
            static_cast<size_t>(converter_.predict_size(frame.width, frame.height)));
          converter_.convert(frame, bgr24.data());

          cv::Mat mat{frame.height, frame.width, CV_8UC3, bgr24.data()};

          if(snap_) takeSnapshot(mat);

#ifdef RECORD
          video->write(mat.clone());
#endif

          if (frame_queue_) {
            frame_queue_->push(mat);
          }
#ifdef RUN_SLAM
          else {
            cv::Mat greyMat;
            cv::cvtColor(mat, greyMat, cv::COLOR_BGR2GRAY);
            api_->addFrameToQueue(greyMat);
            {
              std::unique_lock<std::mutex> lk(api_->getMutex());
              cv::imshow("Pilot view", mat.clone());
              cv::waitKey(1);
            }
          }
#else
          else {
            cv::imshow("Pilot view", mat.clone());
            cv::waitKey(1);
          }
#endif
        } catch (const std::exception& e) {
          utils_log::LogWarn() << "H264 decode failed: " << e.what();
        } catch (...) {
          utils_log::LogWarn() << "H264 decode failed: unknown exception";
        }
      }
      next += consumed;
    }
  }
  catch (...) {
    utils_log::LogErr() << "Error in decoding frame";
  }

  // Keep any unconsumed bytes for the next call (stream-style parsing).
  if (next > 0 && next < total) {
    const size_t remaining = total - next;
    memmove(frame_buffer_, frame_buffer_ + next, remaining);
    first_empty_index = remaining;
  } else if (next >= total) {
    first_empty_index = 0;
  }
}

VideoSocket::~VideoSocket(){
#ifdef RECORD
  video->release();
#endif
  cv::destroyAllWindows();
  socket_.close();
}

void VideoSocket::handleSendCommand(const std::error_code& error, size_t bytes_sent, std::string cmd)
{
  utils_log::LogErr() << "VideoSocket class does not implement handleSendCommand()";
}

void VideoSocket::takeSnapshot(cv::Mat& image){
  snap_ = false;

  time_t rawtime;
  struct tm * timeinfo;
  char buffer [80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (buffer,80,"../snapshots/tello_img_%Y_%m_%d_%H_%M_%S.jpg",timeinfo);
  cv::imwrite(std::string(buffer), image);
  utils_log::LogInfo() << "Picture taken. File " << buffer;
}

void VideoSocket::setSnapshot(){
  snap_ = true;
}
