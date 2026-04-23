#include "vo.h"
#include "frame_queue.hpp"
#include "state_socket.hpp"
#include "tello.hpp"

#include <atomic>
#include <thread>
#include <csignal>

static std::atomic<bool> g_running(true);

void signalHandler(int) { g_running = false; }

int main(int argc, char **argv)
{
  signal(SIGINT,  signalHandler);
  signal(SIGTERM, signalHandler);

  std::string camera_config = "../camera_config.yaml";

  asio::io_service io_service;
  asio::io_service::work work(io_service);
  std::condition_variable cv_run;

  Tello tello(io_service, cv_run, "192.168.10.1", "8889", "11111", "8890",
              camera_config, "../orb_vocab.dbow2");

  FrameQueue frame_queue;
  tello.vs->setFrameQueue(&frame_queue);

  tello.cs->addCommandToQueue("command");
  tello.cs->addCommandToQueue("streamon");
  tello.cs->executeQueue();

  VisualOdometry vo;
  std::atomic<bool> vo_running(true);
  std::thread vo_thread([&](){
    vo.run(frame_queue, *tello.ss, camera_config, vo_running);
  });

  while (g_running) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  vo_running = false;
  if (vo_thread.joinable()) vo_thread.join();

  io_service.stop();
  return 0;
}
