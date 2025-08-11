#ifndef TFLUNA_TFLUNA_DEVICE_HPP_
#define TFLUNA_TFLUNA_DEVICE_HPP_

#include "tca9548a/i2c_device.hpp"
#include "tca9548a/tca9548a.hpp"
#include "tfluna/TFLI2C.hpp"

namespace tfluna {
struct Config {
    uint16_t frameRate;
};
class TFLunaDevice : public tca9548a::I2CDevice {
public:
  TFLunaDevice(std::string i2c_bus, uint8_t sensor_address, Config config_);
  virtual ~TFLunaDevice() = default;
  bool initialize() override;
  bool configure() override;
  tca9548a::msg::SensorData read() override;

private:
  uint8_t sensor_address_;
  tfluna::TFLI2C sensor_;

  Config config_;

private:
};
}; // namespace tfluna

#endif