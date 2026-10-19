#include <zstd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class DepthZstdSender final : public rclcpp::Node
{
public:
  DepthZstdSender() : Node("depth_zstd_sender")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/camera/camera/depth/image_rect_raw");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/camera/transport/depth_zstd");
    preview_topic_ = declare_parameter<std::string>(
      "preview_topic", "/camera/transport/depth_preview/compressed");
    zstd_level_ = declare_parameter<int>("zstd_level", 1);
    max_fps_ = declare_parameter<double>("max_fps", 30.0);
    preview_enabled_ = declare_parameter<bool>("preview_enabled", false);
    preview_quality_ = declare_parameter<int>("preview_quality", 85);
    preview_min_mm_ = declare_parameter<int>("preview_min_mm", 70);
    preview_max_mm_ = declare_parameter<int>("preview_max_mm", 500);

    if (max_fps_ <= 0.0) {
      throw std::runtime_error("max_fps must be greater than zero");
    }
    if (preview_min_mm_ >= preview_max_mm_) {
      throw std::runtime_error("preview_min_mm must be less than preview_max_mm");
    }
    zstd_level_ = std::clamp(zstd_level_, ZSTD_minCLevel(), ZSTD_maxCLevel());
    preview_quality_ = std::clamp(preview_quality_, 1, 100);

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(output_topic_, qos);
    if (preview_enabled_) {
      preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(preview_topic_, qos);
    }

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Expected 16UC1 but received '%s'", msg->encoding.c_str());
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (latest_) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
          }
          latest_ = std::move(msg);
        }
        condition_.notify_one();
      });

    worker_ = std::thread(&DepthZstdSender::worker_loop, this);
    stats_timer_ = create_wall_timer(1s, std::bind(&DepthZstdSender::print_stats, this));

    RCLCPP_INFO(
      get_logger(), "Depth transport: %s -> %s, zstd=%d, max_fps=%.1f, preview=%s",
      input_topic_.c_str(), output_topic_.c_str(), zstd_level_, max_fps_,
      preview_enabled_ ? "on" : "off");
  }

  ~DepthZstdSender() override
  {
    stopping_.store(true, std::memory_order_release);
    condition_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void worker_loop()
  {
    const auto minimum_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / max_fps_));
    auto next_allowed = std::chrono::steady_clock::now();

    while (!stopping_.load(std::memory_order_acquire)) {
      sensor_msgs::msg::Image::ConstSharedPtr msg;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
          return stopping_.load(std::memory_order_acquire) || static_cast<bool>(latest_);
        });
        if (stopping_.load(std::memory_order_acquire)) {
          break;
        }
        msg = std::move(latest_);
      }

      const auto now = std::chrono::steady_clock::now();
      if (now < next_allowed) {
        std::this_thread::sleep_until(next_allowed);
        // Prefer the newest frame that arrived while rate limiting.
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_) {
          msg = std::move(latest_);
          dropped_.fetch_add(1, std::memory_order_relaxed);
        }
      }
      next_allowed = std::chrono::steady_clock::now() + minimum_period;

      encode_and_publish(*msg);
    }
  }

  void encode_and_publish(const sensor_msgs::msg::Image & msg)
  {
    const auto started = std::chrono::steady_clock::now();
    const size_t raw_size = msg.data.size();
    std::vector<uint8_t> encoded(ZSTD_compressBound(raw_size));
    const size_t encoded_size = ZSTD_compress(
      encoded.data(), encoded.size(), msg.data.data(), raw_size, zstd_level_);

    if (ZSTD_isError(encoded_size)) {
      RCLCPP_ERROR(get_logger(), "Zstd: %s", ZSTD_getErrorName(encoded_size));
      return;
    }
    encoded.resize(encoded_size);

    sensor_msgs::msg::CompressedImage output;
    output.header = msg.header;
    std::ostringstream format;
    format << "depth_zstd_v1"
           << ";encoding=" << msg.encoding
           << ";width=" << msg.width
           << ";height=" << msg.height
           << ";step=" << msg.step
           << ";bigendian=" << static_cast<unsigned int>(msg.is_bigendian)
           << ";raw_size=" << raw_size;
    output.format = format.str();
    output.data = std::move(encoded);
    compressed_pub_->publish(std::move(output));

    if (preview_enabled_ && preview_pub_->get_subscription_count() > 0) {
      publish_preview(msg);
    }

    const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    encoded_frames_.fetch_add(1, std::memory_order_relaxed);
    raw_bytes_.fetch_add(raw_size, std::memory_order_relaxed);
    encoded_bytes_.fetch_add(encoded_size, std::memory_order_relaxed);
    encode_us_.fetch_add(static_cast<uint64_t>(elapsed * 1000.0), std::memory_order_relaxed);
  }

  void publish_preview(const sensor_msgs::msg::Image & msg)
  {
    cv::Mat depth(
      static_cast<int>(msg.height), static_cast<int>(msg.width), CV_16UC1,
      const_cast<uint8_t *>(msg.data.data()), static_cast<size_t>(msg.step));
    cv::Mat clipped;
    depth.convertTo(
      clipped, CV_8UC1,
      255.0 / static_cast<double>(preview_max_mm_ - preview_min_mm_),
      -255.0 * static_cast<double>(preview_min_mm_) /
      static_cast<double>(preview_max_mm_ - preview_min_mm_));
    clipped.setTo(0, depth == 0);

    std::vector<uint8_t> jpeg;
    if (!cv::imencode(".jpg", clipped, jpeg, {cv::IMWRITE_JPEG_QUALITY, preview_quality_})) {
      RCLCPP_WARN(get_logger(), "JPEG preview encoding failed");
      return;
    }
    sensor_msgs::msg::CompressedImage preview;
    preview.header = msg.header;
    preview.format = "mono8; jpeg; depth_preview_mm=" +
      std::to_string(preview_min_mm_) + ":" + std::to_string(preview_max_mm_);
    preview.data = std::move(jpeg);
    preview_pub_->publish(std::move(preview));
  }

  void print_stats()
  {
    const uint64_t frames = encoded_frames_.exchange(0);
    const uint64_t raw = raw_bytes_.exchange(0);
    const uint64_t encoded = encoded_bytes_.exchange(0);
    const uint64_t time_us = encode_us_.exchange(0);
    const uint64_t dropped = dropped_.exchange(0);
    if (frames == 0) {
      return;
    }
    const double ratio = encoded == 0 ? 0.0 : static_cast<double>(raw) / encoded;
    const double mbps = static_cast<double>(encoded) * 8.0 / 1'000'000.0;
    const double milliseconds = static_cast<double>(time_us) / frames / 1000.0;
    RCLCPP_INFO(
      get_logger(), "%.0f fps | %.1f Mbps | ratio %.2fx | encode %.2f ms | replaced %lu",
      static_cast<double>(frames), mbps, ratio, milliseconds,
      static_cast<unsigned long>(dropped));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string preview_topic_;
  int zstd_level_;
  double max_fps_;
  bool preview_enabled_;
  int preview_quality_;
  int preview_min_mm_;
  int preview_max_mm_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::mutex mutex_;
  std::condition_variable condition_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_;
  std::thread worker_;
  std::atomic<bool> stopping_{false};
  std::atomic<uint64_t> encoded_frames_{0};
  std::atomic<uint64_t> raw_bytes_{0};
  std::atomic<uint64_t> encoded_bytes_{0};
  std::atomic<uint64_t> encode_us_{0};
  std::atomic<uint64_t> dropped_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthZstdSender>());
  rclcpp::shutdown();
  return 0;
}
