#include <mutex>
#include <condition_variable>
#include <csignal>
#include <atomic>
#include <thread>

#include "utils.hpp"
#include "tello.hpp"
#include "frame_queue.hpp"
#include "vo.h"

// Global flag for signal handling
static std::atomic<bool> g_signal_received(false);
static Tello* g_tello_instance = nullptr;

// Signal handler
void signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        g_signal_received = true;
        utils_log::LogWarn() << "Signal received! Exiting...";
    }
}

#ifdef USE_TERMINAL
#include "command_terminal.hpp"
#endif

#ifdef USE_CONFIG
#include "config_handler.hpp"
#endif

int main(){
  // Register signal handlers for graceful shutdown
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  
  asio::io_service io_service;
  asio::io_service::work work(io_service);

  std::condition_variable cv_run;

  // Shared frame queue for VideoSocket -> VO
  FrameQueue frame_queue;
  std::string camera_config = "../camera_config.yaml";
  std::atomic<bool> vo_running(true);

#ifdef USE_CONFIG

  std::map<std::string, std::unique_ptr<Tello>>  m = handleConfig("../config.yaml", io_service, cv_run);

  if(m.count("0.prime.0") > 0){
    Tello& t = *m["0.prime.0"];
    
    // Pipe decoded frames to the VO queue
    t.vs->setFrameQueue(&frame_queue);

    t.cs->addCommandToQueue("command");
    t.cs->addCommandToQueue("streamon");
    t.cs->executeQueue();

    // Launch VO on a dedicated thread
    VisualOdometry vo;
    std::thread vo_thread([&](){
      vo.run(frame_queue, *t.ss, camera_config, vo_running);
    });
    vo_thread.detach();
  }
  else{
    utils_log::LogErr() << "The requested drone does not exist.";
  }

#else

  Tello t(io_service, cv_run, "192.168.10.1", "8889", "11111", "8890",
          camera_config, "../orb_vocab.dbow2");
  
  g_tello_instance = &t;

  // Pipe decoded frames to the VO queue
  t.vs->setFrameQueue(&frame_queue);

  t.cs->addCommandToQueue("command");
  t.cs->addCommandToQueue("streamon");
  t.cs->executeQueue();

  // Launch VO on a dedicated thread
  VisualOdometry vo;
  std::thread vo_thread([&](){
    vo.run(frame_queue, *t.ss, camera_config, vo_running);
  });

#endif

  {
    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);
    // Wait until signal received or timeout (300 seconds)
    while (!g_signal_received) {
      auto status = cv_run.wait_for(lck, std::chrono::seconds(1));
      if (status == std::cv_status::timeout) {
        // Check signal every second, continue waiting
        continue;
      } else {
        // Condition variable was notified
        break;
      }
    }
  }
  
  utils_log::LogWarn() << "----------- Done -----------";
  utils_log::LogWarn() << "----------- Landing -----------";

  // Stop the VO loop
  vo_running = false;

#ifndef USE_CONFIG
  if (vo_thread.joinable()) vo_thread.join();
#endif

  io_service.stop();
  usleep(1000000);
  utils_log::LogDebug() << "----------- Main thread returns -----------";
  return 0;
}
