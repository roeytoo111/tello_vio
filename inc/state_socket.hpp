#ifndef STATESOCKET_HPP
#define STATESOCKET_HPP

#include "base_socket.hpp"
#include <chrono>
#include <memory>
#include <atomic>

// Forward declarations (removed - no longer needed)

/**
* @class StateSocket
* @brief Prints the state of the tello
*/
class StateSocket : public BaseSocket{
public:

  /**
  * @brief Constructor
  * @param [in] io_service io_service object used to handle all socket communication
  * @param [in] drone_ip ip address of drone
  * @param [in] drone_port port number on the drone
  * @param [in] local_port port on the local machine used to communicate with the drone port mentioned above
  * @return none
  */
  StateSocket(asio::io_service& io_service, 
              const std::string& drone_ip, 
              const std::string& drone_port, 
              const std::string& local_port);
  /**
  * @brief Destructor
  * @return none
  */
  ~StateSocket();

  /**
  * @brief Get the last received battery percentage.
  * @return Battery level in percent (0-100), or -1 if unknown.
  */
  int getLastBattery() const { return last_battery_.load(); }

  /** @brief Get velocity components in cm/s */
  int getVgx() const { return vgx_.load(); }
  int getVgy() const { return vgy_.load(); }
  int getVgz() const { return vgz_.load(); }

  /** @brief Get height in cm */
  int getHeight() const { return height_.load(); }

private:

  virtual void handleResponseFromDrone(const std::error_code& error, size_t bytes_recvd) override;
  virtual void handleSendCommand(const std::error_code& error, size_t bytes_sent, std::string cmd) override;
  
  void parseAndDisplayIMU(const std::string& state_str);
  void updateTelemetry(const std::string& state_str);

  enum{ max_length_ = 1024 };
  bool received_response_ = true;
  char data_[max_length_];
  std::string response_;
  
  // For rate limiting IMU display
  std::chrono::system_clock::time_point last_display_time_;
  static constexpr int display_interval_ms_ = 1000; // Display every 1 second
  
  // For IMU rate tracking
  std::chrono::high_resolution_clock::time_point imu_rate_start_time_;
  size_t imu_message_count_ = 0;
  double current_imu_rate_hz_ = 0.0;
  static constexpr int imu_rate_calculation_interval_ms_ = 2000; // Calculate rate every 2 seconds
  std::chrono::high_resolution_clock::time_point last_rate_calculation_time_;
  
  // For gyroscope estimation from attitude changes
  double last_pitch_ = 0.0;
  double last_roll_ = 0.0;
  double last_yaw_ = 0.0;
  std::chrono::high_resolution_clock::time_point last_attitude_time_;
  bool has_previous_attitude_ = false;

  // Last known battery percentage (updated in parseAndDisplayIMU)
  std::atomic<int> last_battery_{-1};

  // Telemetry for VO scale estimation (updated on every state message)
  std::atomic<int> vgx_{0}, vgy_{0}, vgz_{0};
  std::atomic<int> height_{0};

};

#endif // STATESOCKET_HPP
