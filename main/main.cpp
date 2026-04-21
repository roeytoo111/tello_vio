#include <mutex>
#include <condition_variable>
#include <csignal>
#include <atomic>

#include "utils.hpp"
#include "tello.hpp"

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

#ifdef USE_CONFIG

  std::map<std::string, std::unique_ptr<Tello>>  m = handleConfig("../config.yaml", io_service, cv_run);

  if(m.count("0.prime.0") > 0){
    Tello& t = *m["0.prime.0"];
    
    // Add commands to queue
    t.cs->addCommandToQueue("command");      // Enter SDK mode
    t.cs->addCommandToQueue("streamon");     // Start video
    // t.cs->addCommandToQueue("takeoff");      // Take off
    // t.cs->addCommandToQueue("delay 5"); processingThread     // Wait 5 seconds after takeoff for stability
    // t.cs->addCommandToQueue("rc 0 50 0 0");  // Forward (pitch = 50) - moves forward
    // t.cs->addCommandToQueue("delay 3");      // Keep moving forward for 3 seconds
    // t.cs->addCommandToQueue("rc 0 0 0 0");   // Stop (hover) - all neutral
    // t.cs->addCommandToQueue("delay 2");      // Hover for 2 seconds
    // t.cs->addCommandToQueue("land");         // Land
    
    // Execute the queue
    t.cs->executeQueue();
  }
  else{
    utils_log::LogErr() << "The requested drone does not exist.";
  }

#else

  Tello t(io_service, cv_run, "192.168.10.1", "8889", "11111", "8890", "../camera_config.yaml", "../orb_vocab.dbow2");
  
  // Set global pointer for signal handler
  g_tello_instance = &t;

  // Add commands to queue
  t.cs->addCommandToQueue("command");      // Enter SDK mode
  t.cs->addCommandToQueue("streamon");     // Start video
  // t.cs->addCommandToQueue("takeoff");      // Take off
  // t.cs->addCommandToQueue("delay 5");      // Wait 5 seconds after takeoff for stability
  // t.cs->addCommandToQueue("rc 0 50 0 0");  // Forward (pitch = 50) - moves forward
  // t.cs->addCommandToQueue("delay 3");      // Keep moving forward for 3 seconds
  // t.cs->addCommandToQueue("rc 0 0 0 0");   // Stop (hover) - all neutral
  // t.cs->addCommandToQueue("delay 2");      // Hover for 2 seconds
  // t.cs->addCommandToQueue("land");         // Land
  
  // Execute the queue
  t.cs->executeQueue();

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
  
  // t.cs->exitAllThreads();
  io_service.stop();
  usleep(1000000); // Ensure this is greater than timeout to prevent seg faults
  utils_log::LogDebug() << "----------- Main thread returns -----------";
  return 0;
}
