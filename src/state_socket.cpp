#include "state_socket.hpp"
#include "utils.hpp"
#include <sstream>
#include <map>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <filesystem>

StateSocket::StateSocket(
  asio::io_service& io_service,
  const std::string& drone_ip,
  const std::string& drone_port,
  const std::string& local_port
):
  BaseSocket(io_service, drone_ip, drone_port, local_port)
{
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

    io_thread = std::thread([&]{io_service_.run();});
    io_thread.detach();
    
    // Initialize last display time
    last_display_time_ = std::chrono::system_clock::now();
    
    // Initialize IMU rate tracking
    imu_rate_start_time_ = std::chrono::high_resolution_clock::now();
    last_rate_calculation_time_ = std::chrono::high_resolution_clock::now();
    imu_message_count_ = 0;
    current_imu_rate_hz_ = 0.0;

    // ---- telemetry CSV logging ----
    // Always create a folder and a CSV file so IMU/state is saved even if VO/video fails.
    try {
      const std::filesystem::path out_dir = std::filesystem::absolute("../telemetry");
      std::filesystem::create_directories(out_dir);
      const auto t0 = std::chrono::system_clock::now();
      const std::time_t t0_tt = std::chrono::system_clock::to_time_t(t0);
      std::tm t0_tm = *std::localtime(&t0_tt);
      std::ostringstream fname;
      fname << (out_dir / "tello_state_").string()
            << std::put_time(&t0_tm, "%Y_%m_%d_%H_%M_%S")
            << ".csv";
      telemetry_csv_.open(fname.str(), std::ios::out);
    } catch (...) {
      // Logging is best-effort; do not crash if filesystem is unavailable.
    }

}

void StateSocket::parseAndDisplayIMU(const std::string& state_str)
{
  std::map<std::string, std::string> state_map;
  std::istringstream iss(state_str);
  std::string token;
  
  // Parse semicolon-separated key:value pairs
  while (std::getline(iss, token, ';')) {
    size_t colon_pos = token.find(':');
    if (colon_pos != std::string::npos) {
      std::string key = token.substr(0, colon_pos);
      std::string value = token.substr(colon_pos + 1);
      state_map[key] = value;
    }
  }
  
  // Extract and display IMU-related data
  // Convert accelerometer from Tello units (cm/s²) to m/s²
  std::string agx_str = state_map.count("agx") ? state_map["agx"] : "N/A";
  std::string agy_str = state_map.count("agy") ? state_map["agy"] : "N/A";
  std::string agz_str = state_map.count("agz") ? state_map["agz"] : "N/A";
  
  // Convert accelerometer values for display (cm/s² -> m/s²)
  std::string agx, agy, agz;
  try {
    if (agx_str != "N/A") {
      double agx_val = std::stod(agx_str) / 100.0;  // cm/s² -> m/s²
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << agx_val;
      agx = oss.str();
    } else {
      agx = "N/A";
    }
    if (agy_str != "N/A") {
      double agy_val = std::stod(agy_str) / 100.0;  // cm/s² -> m/s²
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << agy_val;
      agy = oss.str();
    } else {
      agy = "N/A";
    }
    if (agz_str != "N/A") {
      double agz_val = std::stod(agz_str) / 100.0;  // cm/s² -> m/s²
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << agz_val;
      agz = oss.str();
    } else {
      agz = "N/A";
    }
  } catch (const std::exception& e) {
    agx = agx_str;
    agy = agy_str;
    agz = agz_str;
  }
  
  std::string pitch = state_map.count("pitch") ? state_map["pitch"] : "N/A";
  std::string roll = state_map.count("roll") ? state_map["roll"] : "N/A";
  std::string yaw = state_map.count("yaw") ? state_map["yaw"] : "N/A";
  std::string vgx = state_map.count("vgx") ? state_map["vgx"] : "N/A";
  std::string vgy = state_map.count("vgy") ? state_map["vgy"] : "N/A";
  std::string vgz = state_map.count("vgz") ? state_map["vgz"] : "N/A";
  std::string baro = state_map.count("baro") ? state_map["baro"] : "N/A";
  std::string tof = state_map.count("tof") ? state_map["tof"] : "N/A";
  std::string h = state_map.count("h") ? state_map["h"] : "N/A";
  std::string bat = state_map.count("bat") ? state_map["bat"] : "N/A";

  // Update cached battery percentage (if parsable)
  try {
    int bat_val = std::stoi(bat);
    if (bat_val >= 0 && bat_val <= 100) {
      last_battery_.store(bat_val);
    } else {
      last_battery_.store(-1);
    }
  } catch (...) {
    // Keep previous value or mark as unknown
    last_battery_.store(-1);
  }
  
  // Display IMU readings in a formatted way
  utils_log::LogInfo() << "═══════════════════════════════════════════════════════";
  utils_log::LogInfo() << "                    IMU READINGS";
  utils_log::LogInfo() << "═══════════════════════════════════════════════════════";
  utils_log::LogInfo() << "System Status:";
  utils_log::LogInfo() << "  Battery:      " << std::setw(8) << bat << " %";
  utils_log::LogInfo() << "  IMU Rate:     " << std::setw(8) << std::fixed << std::setprecision(2) 
                       << current_imu_rate_hz_ << " Hz";
  utils_log::LogInfo() << "";
  utils_log::LogInfo() << "Accelerometer (Gravity):";
  utils_log::LogInfo() << "  X-axis (agx): " << std::setw(8) << agx << " m/s²";
  utils_log::LogInfo() << "  Y-axis (agy): " << std::setw(8) << agy << " m/s²";
  utils_log::LogInfo() << "  Z-axis (agz): " << std::setw(8) << agz << " m/s²";
  utils_log::LogInfo() << "";
  utils_log::LogInfo() << "Attitude (Euler Angles):";
  utils_log::LogInfo() << "  Pitch:        " << std::setw(8) << pitch << " degrees";
  utils_log::LogInfo() << "  Roll:         " << std::setw(8) << roll << " degrees";
  utils_log::LogInfo() << "  Yaw:          " << std::setw(8) << yaw << " degrees";
  utils_log::LogInfo() << "";
  utils_log::LogInfo() << "Velocity (Ground Speed):";
  utils_log::LogInfo() << "  X (vgx):      " << std::setw(8) << vgx << " cm/s";
  utils_log::LogInfo() << "  Y (vgy):      " << std::setw(8) << vgy << " cm/s";
  utils_log::LogInfo() << "  Z (vgz):      " << std::setw(8) << vgz << " cm/s";
  utils_log::LogInfo() << "";
  utils_log::LogInfo() << "Altitude:";
  utils_log::LogInfo() << "  Barometer:    " << std::setw(8) << baro << " cm";
  utils_log::LogInfo() << "  TOF Sensor:   " << std::setw(8) << tof << " cm";
  utils_log::LogInfo() << "  Height (h):   " << std::setw(8) << h << " cm";
  utils_log::LogInfo() << "═══════════════════════════════════════════════════════";
}

void StateSocket::handleResponseFromDrone(const std::error_code& error, size_t bytes_recvd)
{
  if(!error && bytes_recvd>0){
    response_ = std::string(data_);
    
    // Track IMU rate
    imu_message_count_++;
    auto now_high_res = std::chrono::high_resolution_clock::now();
    auto time_since_rate_calc = std::chrono::duration_cast<std::chrono::milliseconds>(
      now_high_res - last_rate_calculation_time_).count();
    
    // Calculate IMU rate every 2 seconds
    if (time_since_rate_calc >= imu_rate_calculation_interval_ms_) {
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now_high_res - imu_rate_start_time_).count();
      if (elapsed > 0) {
        current_imu_rate_hz_ = (imu_message_count_ * 1000.0) / elapsed;
        imu_message_count_ = 0;
        imu_rate_start_time_ = now_high_res;
      }
      last_rate_calculation_time_ = now_high_res;
    }
    
    updateTelemetry(response_);

    // Log raw state packet to CSV at full rate (~10Hz).
    if (telemetry_csv_.is_open()) {
      std::lock_guard<std::mutex> lk(log_mutex_);
      if (!telemetry_header_written_) {
        telemetry_csv_
          << "t_sec,raw,bat,pitch,roll,yaw,vgx,vgy,vgz,agx,agy,agz,baro,tof,h\n";
        telemetry_header_written_ = true;
      }

      // Parse key:value pairs
      std::map<std::string, std::string> m;
      {
        std::istringstream iss(response_);
        std::string token;
        while (std::getline(iss, token, ';')) {
          size_t colon_pos = token.find(':');
          if (colon_pos == std::string::npos) continue;
          m[token.substr(0, colon_pos)] = token.substr(colon_pos + 1);
        }
      }

      const double t_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

      auto get = [&](const char* k) -> std::string {
        auto it = m.find(k);
        return it == m.end() ? "" : it->second;
      };

      // raw field: quote and replace quotes if any (very rare)
      std::string raw = response_;
      for (char& c : raw) if (c == '"') c = '\'';

      telemetry_csv_ << std::fixed << std::setprecision(6) << t_sec << ","
                     << "\"" << raw << "\"" << ","
                     << get("bat") << ","
                     << get("pitch") << ","
                     << get("roll") << ","
                     << get("yaw") << ","
                     << get("vgx") << ","
                     << get("vgy") << ","
                     << get("vgz") << ","
                     << get("agx") << ","
                     << get("agy") << ","
                     << get("agz") << ","
                     << get("baro") << ","
                     << get("tof") << ","
                     << get("h") << "\n";
      telemetry_csv_.flush();
    }

    // Rate limit IMU display to once per second (state updates ~10 times per second)
    auto now = std::chrono::system_clock::now();
    auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_display_time_).count();
    
    if (time_since_last >= display_interval_ms_) {
      parseAndDisplayIMU(response_);
      last_display_time_ = now;
    }
    
    // Also keep the old format for reference (commented out)
    // std::replace(response_.begin(), response_.end(), ';', '\n');
    // std::cout << "Status: \n" << response_ << std::endl;
  }
  else{
    // utils_log::LogDebug() << "Error/Nothing received" ;
  }

  socket_.async_receive_from(
    asio::buffer(data_, max_length_),
    endpoint_,
    [&](const std::error_code& error, size_t bytes_recvd)
    {return handleResponseFromDrone(error, bytes_recvd);});
    // [&](auto... args){return handleResponseFromDrone(args...);});
}

StateSocket::~StateSocket(){
  if (telemetry_csv_.is_open()) telemetry_csv_.close();
  socket_.close();
}

void StateSocket::handleSendCommand(const std::error_code& error, size_t bytes_sent, std::string cmd)
{
  std::cout << "StateSocket class does not implement handleSendCommand()" << std::endl;
}

void StateSocket::updateTelemetry(const std::string& state_str) {
  std::istringstream iss(state_str);
  std::string token;
  while (std::getline(iss, token, ';')) {
    size_t colon_pos = token.find(':');
    if (colon_pos == std::string::npos) continue;
    std::string key = token.substr(0, colon_pos);
    std::string value = token.substr(colon_pos + 1);
    try {
      if (key == "vgx") vgx_.store(std::stoi(value));
      else if (key == "vgy") vgy_.store(std::stoi(value));
      else if (key == "vgz") vgz_.store(std::stoi(value));
      else if (key == "h") height_.store(std::stoi(value));
    } catch (...) {}
  }
}
