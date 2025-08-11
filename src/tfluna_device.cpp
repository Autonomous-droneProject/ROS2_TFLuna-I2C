#include "tfluna/tfluna_device.hpp"

namespace tfluna {
TFLunaDevice::TFLunaDevice(std::string i2c_bus, uint8_t sensor_address,
                             Config config)
    : sensor_(i2c_bus), config_(config) {}
bool TFLunaDevice::initialize() { 
    return sensor_.Soft_Reset();
}

bool TFLunaDevice::configure() {
  sensor_.Set_Frame_Rate(config_.frameRate);

}
tca9548a::msg::SensorData TFLunaDevice::read() {
    tca9548a::msg::SensorData message;
    int16_t tfDist;
    int16_t tfFlux;
    int16_t tfTemp;
    bool read_success = sensor_.getData(tfDist, tfFlux, tfTemp);

    if (read_success) {
        message.header.stamp = rclcpp::Clock().now();
        message.device_name = "tfluna_sensor";
        
        diagnostic_msgs::msg::KeyValue distance_kv;
        distance_kv.key = "distance_mm";
        distance_kv.value = std::to_string(tfDist);
        message.values.push_back(distance_kv);

        diagnostic_msgs::msg::KeyValue flux_kv;
        flux_kv.key = "flux_arbitrary";
        flux_kv.value = std::to_string(tfFlux);
        message.values.push_back(flux_kv);

        diagnostic_msgs::msg::KeyValue temp_kv;
        temp_kv.key = "temp_c";
        temp_kv.value = std::to_string(tfTemp);
        message.values.push_back(temp_kv);
    } else {
        // Return an empty message with a null timestamp to signal failure
        message.header.stamp = rclcpp::Time(0, 0);
    }

    return message;
}
}