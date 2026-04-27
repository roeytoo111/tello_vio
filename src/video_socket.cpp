#include <queue>
#include <mutex>
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <new>

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
  static uint64_t last_decoded_frames = 0;
  static uint64_t last_decode_failures = 0;

  if (error) {
    utils_log::LogWarn() << "VideoSocket error receiving packet: " << error.message();
  } else if (bytes_recvd > 0) {
    last_pkt_len_.store(bytes_recvd);
    const size_t n = std::min<size_t>(8, bytes_recvd);
    for (size_t i = 0; i < n; i++) {
      last_pkt_prefix_[i].store(static_cast<uint8_t>(data_[i]));
    }
    packet_count++;
    auto now = std::chrono::steady_clock::now();
    auto time_since_log = std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count();
    
    // Log packet reception every 5 seconds for debugging
    if (time_since_log >= 5) {
      utils_log::LogInfo() << "VideoSocket: Received " << packet_count << " packets in last 5 seconds";
      const uint64_t df = decoded_frames_.load();
      const uint64_t ff = decode_failures_.load();
      const uint64_t df_delta = df - last_decoded_frames;
      const uint64_t ff_delta = ff - last_decode_failures;
      last_decoded_frames = df;
      last_decode_failures = ff;

      const int w = last_frame_w_.load();
      const int h = last_frame_h_.load();
      std::ostringstream oss;
      oss << std::hex << std::setfill('0');
      const size_t pkt_len = last_pkt_len_.load();
      const size_t prefix_n = std::min<size_t>(8, pkt_len);
      for (size_t i = 0; i < prefix_n; i++) {
        oss << std::setw(2) << static_cast<int>(last_pkt_prefix_[i].load());
        if (i + 1 < prefix_n) oss << " ";
      }
      utils_log::LogInfo() << "VideoSocket: Decoded " << df_delta << " frames, "
                           << ff_delta << " decode failures in last 5 seconds"
                           << " (last frame " << w << "x" << h << ")"
                           << " (last pkt len " << pkt_len << ", prefix [" << oss.str() << "])";
      packet_count = 0;
      last_log_time = now;
    }
  }

  // Feed the decoder as a continuous bytestream. Do not try to guess "frame"
  // boundaries from UDP packet sizes; let the H264 parser do that.
  if (bytes_recvd > 0) {
    decodeBytes(reinterpret_cast<const unsigned char*>(data_), bytes_recvd);
  }

  socket_.async_receive_from(
    asio::buffer(data_, max_length_),
    endpoint_,
    [&](const std::error_code& error, size_t bytes_recvd)
    {return handleResponseFromDrone(error, bytes_recvd);});
    // [&](auto... args){return handleResponseFromDrone(args...);});
}

void VideoSocket::decodeBytes(const unsigned char* data, size_t len)
{
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
  last_decode_attempt_ms_.store(static_cast<int64_t>(now_ms));

  // Append payload to stream buffer (bounded). We avoid vector::erase on every
  // loop because it is O(n) and can lead to high churn / bad_alloc.
  constexpr size_t kMaxStreamBuf = 256 * 1024;
  if (len > 0) {
    // Enforce a hard cap *before* inserting new bytes (prevents std::bad_alloc
    // during insert/reallocation when the decoder stalls on corrupted input).
    const size_t live = (h264_stream_buf_.size() >= h264_stream_off_)
                          ? (h264_stream_buf_.size() - h264_stream_off_)
                          : h264_stream_buf_.size();
    if (live + len > kMaxStreamBuf) {
      utils_log::LogWarn() << "H264 stream buffer would exceed " << kMaxStreamBuf
                           << " bytes; resetting decoder and dropping buffered data.";
      h264_stream_buf_.clear();
      h264_stream_off_ = 0;
      have_sps_ = have_pps_ = have_idr_ = false;
      consecutive_decode_failures_ = 0;
      decoder_.~H264Decoder();
      new (&decoder_) H264Decoder();
      converter_.~ConverterRGB24();
      new (&converter_) ConverterRGB24();
      bgr24_buf_.clear();
    }
    if (h264_stream_buf_.capacity() < 256 * 1024) {
      h264_stream_buf_.reserve(256 * 1024);
    }
    h264_stream_buf_.insert(h264_stream_buf_.end(), data, data + len);
  }

  // Align buffer to the first Annex-B start code. Some setups prepend junk
  // bytes before the H264 stream; we must discard them or the parser will
  // never output frames.
  auto find_start_code_in_buf = [&](const unsigned char* p, size_t n) -> size_t {
    if (n < 4) return static_cast<size_t>(-1);
    for (size_t i = 0; i + 3 < n; i++) {
      // 00 00 01
      if (p[i] == 0x00 && p[i + 1] == 0x00 && p[i + 2] == 0x01) return i;
      // 00 00 00 01
      if (i + 3 < n && p[i] == 0x00 && p[i + 1] == 0x00 && p[i + 2] == 0x00 && p[i + 3] == 0x01) return i;
    }
    return static_cast<size_t>(-1);
  };

  if (!h264_stream_buf_.empty() && h264_stream_off_ < h264_stream_buf_.size()) {
    const unsigned char* base = h264_stream_buf_.data() + h264_stream_off_;
    const size_t avail = h264_stream_buf_.size() - h264_stream_off_;
    const size_t idx = find_start_code_in_buf(base, avail);
    if (idx == static_cast<size_t>(-1)) {
      // No start code yet; keep bounded and wait for more.
      // If buffer gets too large without any start code, drop it.
      if (avail > 64 * 1024) {
        utils_log::LogWarn() << "No H264 start code found in buffer; dropping " << avail << " bytes";
        h264_stream_buf_.clear();
        h264_stream_off_ = 0;
      }
      return;
    }
    if (idx > 0) {
      h264_stream_off_ += idx;
    }
  }

  // Compact occasionally if offset grows.
  if (h264_stream_off_ > 0 && h264_stream_off_ > 64 * 1024) {
    const size_t remaining = h264_stream_buf_.size() - h264_stream_off_;
    memmove(h264_stream_buf_.data(), h264_stream_buf_.data() + h264_stream_off_, remaining);
    h264_stream_buf_.resize(remaining);
    h264_stream_off_ = 0;
  }

  // Hard cap: keep last ~256KB of stream data. If we hit it without decoding,
  // reset decoder state (usually means we started mid-stream without SPS/PPS).
  if (h264_stream_buf_.size() - h264_stream_off_ > kMaxStreamBuf) {
    utils_log::LogWarn() << "H264 stream buffer exceeded " << kMaxStreamBuf
                         << " bytes without producing frames; resetting decoder.";
    h264_stream_buf_.clear();
    h264_stream_off_ = 0;
    have_sps_ = false;
    have_pps_ = false;
    have_idr_ = false;
    consecutive_decode_failures_ = 0;
    // Reinitialize decoder and converter in-place.
    decoder_.~H264Decoder();
    new (&decoder_) H264Decoder();
    converter_.~ConverterRGB24();
    new (&converter_) ConverterRGB24();
    bgr24_buf_.clear();
    return;
  }

  auto find_start_code_at = [&](const unsigned char* p, size_t n, size_t start) -> size_t {
    if (n < 4) return static_cast<size_t>(-1);
    for (size_t i = start; i + 3 < n; i++) {
      if (p[i] == 0x00 && p[i + 1] == 0x00 && p[i + 2] == 0x01) return i;
      if (i + 3 < n && p[i] == 0x00 && p[i + 1] == 0x00 && p[i + 2] == 0x00 && p[i + 3] == 0x01) return i;
    }
    return static_cast<size_t>(-1);
  };

  size_t next = 0;
  try {
    const unsigned char* base = h264_stream_buf_.data() + h264_stream_off_;
    const size_t available = h264_stream_buf_.size() - h264_stream_off_;

    // Extract NAL units (Annex-B) and feed them one by one. We only feed NALs
    // that are fully delimited by a *next* start code; otherwise we may feed a
    // truncated NAL and poison the decoder.
    size_t nal_start = 0;
    while (nal_start < available) {
      // Find current start code
      size_t sc = find_start_code_at(base, available, nal_start);
      if (sc == static_cast<size_t>(-1)) break;

      // Find next start code to determine NAL end
      size_t next_sc = find_start_code_at(base, available, sc + 3);
      if (next_sc == static_cast<size_t>(-1)) {
        // Incomplete last NAL; keep remainder for next UDP packets.
        break;
      }

      const size_t nal_size = next_sc - sc;
      if (nal_size > 0) {
        try {
          // Track SPS/PPS/IDR gating.
          const size_t hdr_off = (base[sc + 2] == 0x01) ? 3 : 4; // start code len
          if (sc + hdr_off < available) {
            const uint8_t nal_type = base[sc + hdr_off] & 0x1F;
            if (nal_type == 7) have_sps_ = true;
            else if (nal_type == 8) have_pps_ = true;
            else if (nal_type == 5) have_idr_ = true;
          }

          decoder_.feed_packet(base + sc, nal_size);
          while (decoder_.try_decode()) {
            const AVFrame& frame = decoder_.get_frame();
            if (frame.width <= 0 || frame.height <= 0 || frame.width > 4000 || frame.height > 4000) {
              decode_failures_.fetch_add(1);
              utils_log::LogWarn() << "Skipping invalid decoded frame dims: "
                                   << frame.width << "x" << frame.height;
              continue;
            }

            const int predicted = converter_.predict_size(frame.width, frame.height);
            if (predicted <= 0 || predicted > 100 * 1024 * 1024) { // 100MB cap
              decode_failures_.fetch_add(1);
              utils_log::LogWarn() << "Skipping frame due to invalid predicted size: " << predicted;
              continue;
            }

            bgr24_buf_.resize(static_cast<size_t>(predicted));
            converter_.convert(frame, bgr24_buf_.data());

            cv::Mat mat{frame.height, frame.width, CV_8UC3, bgr24_buf_.data()};
            decoded_frames_.fetch_add(1);
            last_frame_w_.store(frame.width);
            last_frame_h_.store(frame.height);
            const auto now_ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch()).count();
            last_frame_ms_.store(static_cast<int64_t>(now_ms2));

            if(snap_) takeSnapshot(mat);

#ifdef RECORD
            video->write(mat.clone());
#endif

            if (have_sps_ && have_pps_ && have_idr_) {
              consecutive_decode_failures_ = 0;
              if (frame_queue_) {
                frame_queue_->push(mat);
                cv::imshow("Pilot view", mat);
                cv::waitKey(1);
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
            }
          }
        } catch (const std::exception& e) {
          decode_failures_.fetch_add(1);
          consecutive_decode_failures_++;
          utils_log::LogWarn() << "H264 decode failed: " << e.what();

          if (consecutive_decode_failures_ > 50) {
            utils_log::LogWarn() << "Too many decode failures; waiting for next keyframe (SPS/PPS/IDR).";
            have_sps_ = have_pps_ = have_idr_ = false;
            consecutive_decode_failures_ = 0;
          }
        } catch (...) {
          decode_failures_.fetch_add(1);
          consecutive_decode_failures_++;
          utils_log::LogWarn() << "H264 decode failed: unknown exception";
        }
      }

      nal_start = next_sc;
      next = next_sc;
    }
  }
  catch (...) {
    utils_log::LogErr() << "Error in decoding frame";
  }

  // Advance stream offset by consumed bytes; remainder stays for next packets.
  if (next > 0) {
    h264_stream_off_ += next;
    if (h264_stream_off_ >= h264_stream_buf_.size()) {
      h264_stream_buf_.clear();
      h264_stream_off_ = 0;
    }
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
