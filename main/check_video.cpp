// Standalone tool to check that the Tello video stream is being received and decoded.
// Builds as a separate executable so you can run it without affecting the main app / VO.

#include <atomic>
#include <csignal>
#include <condition_variable>
#include <thread>

#include "tello.hpp"
#include "utils.hpp"

static std::atomic<bool> g_stop(false);

static void signalHandler(int sig) {
  if (sig == SIGINT || sig == SIGTERM) {
    g_stop.store(true);
    utils_log::LogWarn() << "Signal received, stopping video check...";
  }
}

int main() {
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  asio::io_service io_service;
  asio::io_service::work work(io_service);
  std::condition_variable cv_run;

  const std::string camera_config = "../camera_config.yaml";

  // Construct the Tello object which creates CommandSocket + VideoSocket + StateSocket.
  // VideoSocket will open a window ("Pilot view") and display decoded frames by default.
  Tello t(io_service, cv_run, "192.168.10.1", "8889", "11111", "8890",
          camera_config, "../orb_vocab.dbow2");

  // Enter SDK mode and restart the video stream so we receive SPS/PPS at the beginning.
  t.cs->addCommandToQueue("command");
  t.cs->addCommandToQueue("streamoff");
  t.cs->addCommandToQueue("streamon");
  t.cs->executeQueue();

  utils_log::LogInfo() << "Video check running. You should see the 'Pilot view' window.";
  utils_log::LogInfo() << "Press Ctrl+C to stop.";

  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Best-effort cleanup: stop the stream to reduce drone workload.
  try {
    if (t.cs) {
      t.cs->stopQueueExecution();
      t.cs->clearQueue();
      t.cs->addCommandToQueue("streamoff");
      t.cs->executeQueue();
    }
  } catch (...) {
  }

  // Give sockets a moment to flush/close.
  io_service.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return 0;
}

