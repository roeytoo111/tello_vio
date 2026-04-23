#ifndef FRAME_QUEUE_HPP
#define FRAME_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <opencv2/core/core.hpp>

class FrameQueue {
public:
  void push(const cv::Mat& frame) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= max_size_) {
        queue_.pop();
      }
      queue_.push(frame.clone());
    }
    cv_.notify_one();
  }

  bool pop(cv::Mat& frame, int timeout_ms = 1000) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                     [this] { return !queue_.empty(); })) {
      frame = queue_.front();
      queue_.pop();
      return true;
    }
    return false;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!queue_.empty()) queue_.pop();
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::queue<cv::Mat> queue_;
  static constexpr size_t max_size_ = 10;
};

#endif // FRAME_QUEUE_HPP
