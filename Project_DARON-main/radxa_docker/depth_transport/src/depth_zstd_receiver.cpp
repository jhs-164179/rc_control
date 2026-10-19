#include <zstd.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class DepthZstdReceiver final : public rclcpp::Node
{
public:
  DepthZstdReceiver() : Node("depth_zstd_receiver")
  {
    const auto input = declare_parameter<std::string>(
      "input_topic", "/camera/transport/depth_zstd");
    const auto output = declare_parameter<std::string>(
      "output_topic", "/camera/transport/depth_raw");

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::Image>(output, qos);
    subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input, qos,
      std::bind(&DepthZstdReceiver::callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Depth receiver: %s -> %s", input.c_str(), output.c_str());
  }

private:
  static std::unordered_map<std::string, std::string> parse_format(const std::string & format)
  {
    std::unordered_map<std::string, std::string> fields;
    std::stringstream stream(format);
    std::string token;
    std::getline(stream, token, ';');
    fields["codec"] = token;
    while (std::getline(stream, token, ';')) {
      const auto position = token.find('=');
      if (position != std::string::npos) {
        fields[token.substr(0, position)] = token.substr(position + 1);
      }
    }
    return fields;
  }

  static uint32_t as_u32(
    const std::unordered_map<std::string, std::string> & fields, const std::string & key)
  {
    const auto found = fields.find(key);
    if (found == fields.end()) {
      throw std::runtime_error("missing field: " + key);
    }
    return static_cast<uint32_t>(std::stoul(found->second));
  }

  void callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    try {
      const auto fields = parse_format(msg->format);
      if (fields.at("codec") != "depth_zstd_v1") {
        throw std::runtime_error("unsupported codec: " + fields.at("codec"));
      }
      const uint32_t width = as_u32(fields, "width");
      const uint32_t height = as_u32(fields, "height");
      const uint32_t step = as_u32(fields, "step");
      const uint32_t raw_size = as_u32(fields, "raw_size");
      if (raw_size != static_cast<uint64_t>(step) * height || raw_size > 64U * 1024U * 1024U) {
        throw std::runtime_error("invalid raw_size");
      }

      sensor_msgs::msg::Image output;
      output.header = msg->header;
      output.height = height;
      output.width = width;
      output.encoding = fields.at("encoding");
      output.is_bigendian = static_cast<uint8_t>(as_u32(fields, "bigendian"));
      output.step = step;
      output.data.resize(raw_size);

      const size_t result = ZSTD_decompress(
        output.data.data(), output.data.size(), msg->data.data(), msg->data.size());
      if (ZSTD_isError(result)) {
        throw std::runtime_error(std::string("Zstd: ") + ZSTD_getErrorName(result));
      }
      if (result != raw_size) {
        throw std::runtime_error("decompressed size mismatch");
      }
      publisher_->publish(std::move(output));
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthZstdReceiver>());
  rclcpp::shutdown();
  return 0;
}
