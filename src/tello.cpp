#include <fstream>
#include <cstdlib>
#include <opencv2/highgui/highgui.hpp>

#include "tello.hpp"

Tello::Tello(
#ifdef USE_BOOST
    boost::asio::io_service& io_service,
#else
    asio::io_service& io_service,
#endif
std::condition_variable& cv_run,
const std::string drone_ip,
const std::string local_drone_port,
const std::string local_video_port,
const std::string local_state_port,
const std::string camera_config_file,
const std::string vocabulary_file,
const int n_retries,
const int timeout,
const std::string load_map_db_path,
const std::string save_map_db_path,
const std::string mask_img_path,
bool load_map,
bool continue_mapping,
float scale,
const std::string sequence_file
):
io_service_(io_service),
cv_run_(cv_run)
{
  cs = std::make_unique<CommandSocket>(io_service, drone_ip, "8889", local_drone_port, n_retries, timeout);
  vs = std::make_unique<VideoSocket>(io_service,  "0.0.0.0", "11111", local_video_port,
    run_, camera_config_file, vocabulary_file, load_map_db_path, save_map_db_path,
    mask_img_path, load_map, continue_mapping, scale, nullptr);
  ss = std::make_unique<StateSocket>(io_service, "0.0.0.0", "8890", local_state_port);

#ifdef USE_TERMINAL
  term_ = std::make_unique<Terminal>(run_);
  term_thread_worker_ = std::thread([&]{term_->terminalWorker();});
  term_thread_fetch_ = std::thread([&]{terminalToCommandThread();});
  term_thread_worker_.detach();
  term_thread_fetch_.detach();
#endif // TERMINAL

  // If using joystick, joystick should be initailized before adding to
  // command queue; safety.
  readSequence(sequence_file);
  
  // Start trajectory visualization thread
  trajectory_viz_thread_ = std::thread([&]{trajectoryVisualizationThread();});
  trajectory_viz_thread_.detach();

}

#ifdef USE_TERMINAL
void Tello::terminalToCommandThread(){
  while(run_)
  {
    usleep(1000);
    if(term_->hasCommnad()){
      terminalToCommand(term_->getCommand());
    }
  }
  std::cout << "----------- Terminal to command thread exits -----------" << std::endl;
}



void Tello::terminalToCommand(const std::string& cmd) {
  if (cmd.empty()) return;

  std::stringstream ss(cmd);
  std::string word;
  std::vector<std::string> cmd_v;
  while (ss >> word) {
      cmd_v.push_back(word);
  }

  if (cmd_v.empty()) return;

  if (cmd_v[0] == "queue" && cmd_v.size() >= 2) {
      auto action = term_->convertToEnum(cmd_v[1]);
      
      switch (action) {
          case ADD:
              if (cmd_v.size() >= 3) cs->addCommandToQueue(cmd_v[2]);
              break;
          case START:     cs->executeQueue(); break;
          case STOP:      cs->stopQueueExecution(); break;
          case CLEAR:     cs->clearQueue(); break;
          case ALLOW_AUTO_LAND:    cs->allowAutoLand(); break; //
          case DO_NOT_AUTO_LAND:   cs->doNotAutoLand(); break; // 
          default:
              utils_log::LogErr() << "Unknown or incomplete queue command";
              break;
      }
  } 
  else {
      // 
      if (cs->isExecutingQueue()) {
          cs->stopQueueExecution();
      }
      
      std::lock_guard<std::mutex> lk(m_commandMutex); 
      cs->sendCommand(cmd);
  }
}
#endif // TERMINAL


// Joystick support removed: jsToCommand* no longer used.

void Tello::readSequence(const std::string& file){
  if(!file.empty()){
    std::ifstream ifile(file);
    if(!ifile.is_open()){
      utils_log::LogErr() << "File does not exist";
      return;
    }
    std::string line;
    while(std::getline(ifile, line)){
      //  NOTE: add check here?
      utils_log::LogDebug() << "Adding to queue from file: [" << line << "]";
      cs->addCommandToQueue(line);
    }
  }

}

void Tello::trajectoryVisualizationThread() {
  utils_log::LogInfo() << "Trajectory visualization thread started";
  
  // Wait a bit for VIO to initialize
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  auto last_pose_log_time = std::chrono::steady_clock::now();
  auto last_rc_send_time = std::chrono::steady_clock::now();
  last_key_press_time_ = std::chrono::steady_clock::now();
  constexpr int pose_log_interval_ms = 1000; // Log pose every second
  constexpr int rc_send_interval_ms = 100;   // Send RC commands every 100ms (10 Hz)
  constexpr int rc_reset_timeout_ms = 200;   // Reset RC to zero after 200ms of no key press
  
  while (run_) {
    // Continuously send RC commands at 10 Hz (for smooth control)
    auto now = std::chrono::steady_clock::now();
    auto time_since_rc = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_rc_send_time).count();
    auto time_since_key = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_key_press_time_).count();
    
    if (time_since_rc >= rc_send_interval_ms) {
      std::lock_guard<std::mutex> lock(keyboard_rc_mutex_);
      
      // Auto-reset RC values to zero if no key pressed for timeout period
      if (time_since_key >= rc_reset_timeout_ms) {
        if (rc_left_right_ != 0 || rc_forward_back_ != 0 || rc_up_down_ != 0 || rc_yaw_ != 0) {
          rc_left_right_ = 0;
          rc_forward_back_ = 0;
          rc_up_down_ = 0;
          rc_yaw_ = 0;
        }
      }
      
      // Always send RC command (even if all zeros, to maintain hover)
      updateRCFromKeyboard();
      last_rc_send_time = now;
    }
    
    // Update display at ~30 FPS and check for keyboard input
    int key = cv::waitKey(33); // ~30 FPS (non-blocking)
    if (key != -1) {
      keyboardToCommand(key);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }
  
  utils_log::LogInfo() << "Trajectory visualization thread exiting";
}

void Tello::keyboardToCommand(int key) {
  // Convert OpenCV key code to ASCII (handle special keys)
  char key_char = key & 0xFF;
  
  // Update RC values based on key press
  std::lock_guard<std::mutex> lock(keyboard_rc_mutex_);
  
  // Update last key press time
  last_key_press_time_ = std::chrono::steady_clock::now();
  
  const int rc_speed = 50; // Default movement speed (0-100)
  
  switch (key_char) {
    // Movement keys
    case 'w':
    case 'W':
      rc_forward_back_ = rc_speed; // Forward
      utils_log::LogInfo() << "Keyboard: Forward (W)";
      break;
    case 's':
    case 'S':
      rc_forward_back_ = -rc_speed; // Backward
      break;
    case 'a':
    case 'A':
      rc_left_right_ = -rc_speed; // Left
      break;
    case 'd':
    case 'D':
      rc_left_right_ = rc_speed; // Right
      break;
    case 'q':
    case 'Q':
      rc_yaw_ = -rc_speed; // Rotate left
      break;
    case 'e':
    case 'E':
      rc_yaw_ = rc_speed; // Rotate right
      break;
    case 'r':
    case 'R':
      rc_up_down_ = rc_speed; // Up
      break;
    case 'f':
    case 'F':
      rc_up_down_ = -rc_speed; // Down
      break;
    
    // Stop/hover
    case ' ':
      rc_left_right_ = 0;
      rc_forward_back_ = 0;
      rc_up_down_ = 0;
      rc_yaw_ = 0;
      updateRCFromKeyboard();
      break;
    
    // Command keys
    case 't':
    case 'T':
      if (cs->isExecutingQueue()) cs->stopQueueExecution();
      cs->sendCommand("takeoff");
      utils_log::LogInfo() << "Keyboard: Takeoff";
      break;
    case 'l':
    case 'L':
      if (cs->isExecutingQueue()) cs->stopQueueExecution();
      cs->sendCommand("land");
      utils_log::LogInfo() << "Keyboard: Land";
      break;
    case 'x':
    case 'X':
      if (cs->isExecutingQueue()) cs->stopQueueExecution();
      cs->sendCommand("emergency");
      utils_log::LogInfo() << "Keyboard: Emergency stop";
      break;
    case 'c':
    case 'C':
      if (cs->isExecutingQueue()) cs->stopQueueExecution();
      cs->sendCommand("command");
      utils_log::LogInfo() << "Keyboard: Enter SDK mode";
      break;
    case 'v':
    case 'V':
      if (cs->isExecutingQueue()) cs->stopQueueExecution();
      cs->sendCommand("streamon");
      utils_log::LogInfo() << "Keyboard: Start video stream";
      break;
    case 'b':
    case 'B':
      if (cs->isExecutingQueue()) cs->stopQueueExecution();
      cs->sendCommand("streamoff");
      utils_log::LogInfo() << "Keyboard: Stop video stream";
      break;
    
    // Special keys (ESC, arrow keys, etc.)
    case 27: // ESC
      if (cs->isExecutingQueue()) cs->stopQueueExecution();
      cs->sendCommand("land");
      utils_log::LogInfo() << "Keyboard: ESC - Landing and exiting";
      run_ = false;
      break;
    
    default:
      // Check for arrow keys (OpenCV special key codes)
      if (key == 82 || key == 65362) { // Up arrow
        rc_forward_back_ = rc_speed;
        utils_log::LogInfo() << "Keyboard: Forward (Up Arrow)";
      } else if (key == 84 || key == 65364) { // Down arrow
        rc_forward_back_ = -rc_speed;
        utils_log::LogInfo() << "Keyboard: Backward (Down Arrow)";
      } else if (key == 81 || key == 65361) { // Left arrow
        rc_left_right_ = -rc_speed;
        utils_log::LogInfo() << "Keyboard: Left (Left Arrow)";
      } else if (key == 83 || key == 65363) { // Right arrow
        rc_left_right_ = rc_speed;
        utils_log::LogInfo() << "Keyboard: Right (Right Arrow)";
      } else {
        return; // Unknown key, don't update RC
      }
      break;
  }
  
  // Note: RC commands are sent continuously in trajectoryVisualizationThread
  // at 10 Hz, so we don't need to send immediately here
}

void Tello::updateRCFromKeyboard() {
  // Don't lock here - caller should already have lock
  
  // Stop queue execution if running
  if (cs->isExecutingQueue()) {
    cs->stopQueueExecution();
  }
  
  // Build RC command: rc left_right forward_back up_down yaw
  std::string cmd = "rc " +
    std::to_string(rc_left_right_) + " " +
    std::to_string(rc_forward_back_) + " " +
    std::to_string(rc_up_down_) + " " +
    std::to_string(rc_yaw_);
  
  cs->sendCommand(cmd);
  
  // Log RC command for debugging
  static int log_counter = 0;
  if (++log_counter % 10 == 0) {  // Log every 10th command to avoid spam
    utils_log::LogDebug() << "RC: [" << rc_left_right_ << ", " << rc_forward_back_ 
                          << ", " << rc_up_down_ << ", " << rc_yaw_ << "]";
  }
}

Tello::~Tello(){
  run_ = false;
  // Wait for visualization thread to finish
  if (trajectory_viz_thread_.joinable()) {
    trajectory_viz_thread_.join();
  }
  
  // Note: Trajectory saving is now handled by test_vo.cpp or other VO implementations
  usleep(1000000);
}
