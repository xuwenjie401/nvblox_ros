#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nvblox/core/cuda_stream.h>
#include <nvblox/core/indexing.h>
#include <nvblox/core/types.h>
#include <nvblox/map/blox.h>
#include <nvblox/map/common_names.h>
#include <nvblox/map/layer.h>
#include <nvblox/map/voxels.h>
#include <nvblox/mapper/mapper.h>
#include <nvblox/sensors/camera.h>
#include <nvblox/sensors/image.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace nvblox_ros {
namespace {

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

template <typename T>
T readUnaligned(const uint8_t* ptr) {
  T value;
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

bool isDepthEncoding(const std::string& encoding) {
  return encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
         encoding == sensor_msgs::image_encodings::TYPE_16UC1;
}

bool isMaskEncoding(const std::string& encoding) {
  return encoding == sensor_msgs::image_encodings::MONO8 ||
         encoding == sensor_msgs::image_encodings::TYPE_8UC1;
}

std::string normalizeFrame(std::string frame) {
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

bool frameMatches(const std::string& actual, const std::string& expected) {
  const std::string actual_norm = normalizeFrame(actual);
  const std::string expected_norm = normalizeFrame(expected);
  if (actual_norm == expected_norm) {
    return true;
  }
  return actual_norm.size() > expected_norm.size() &&
         actual_norm.compare(actual_norm.size() - expected_norm.size(),
                             expected_norm.size(), expected_norm) == 0 &&
         actual_norm[actual_norm.size() - expected_norm.size() - 1] == '/';
}

int64_t stampToNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<int64_t>(stamp.nanosec);
}

int64_t secondsToNanoseconds(const double seconds) {
  constexpr double kNanosecondsPerSecond = 1.0e9;
  return static_cast<int64_t>(seconds * kNanosecondsPerSecond);
}

double stampDeltaSec(const builtin_interfaces::msg::Time& a,
                     const builtin_interfaces::msg::Time& b) {
  return std::abs(static_cast<double>(stampToNanoseconds(a) -
                                      stampToNanoseconds(b)) *
                  1.0e-9);
}

}  // namespace

class TsdfOnlyNode : public rclcpp::Node {
 public:
  explicit TsdfOnlyNode(const rclcpp::NodeOptions& options)
      : Node("tsdf_only_node", options), config_(loadConfig()) {
    mapper_ = std::make_unique<nvblox::Mapper>(
        config_.voxel_size_m,
        nvblox::BlockMemoryPoolParams(nvblox::MemoryType::kUnified),
        nvblox::ProjectiveLayerType::kTsdf);
    mapper_->tsdf_integrator().max_integration_distance_m(
        config_.max_integration_distance_m);
    mapper_->tsdf_integrator().truncation_distance_vox(
        config_.truncation_distance_vox);
    mapper_->tsdf_integrator().max_weight(config_.max_weight);

    camera_ = nvblox::Camera(config_.fx, config_.fy, config_.cx, config_.cy,
                             config_.width, config_.height);

    const auto input_qos =
        rclcpp::QoS(static_cast<size_t>(config_.input_queue_size)).reliable();
    depth_sub_.subscribe(this, config_.depth_topic,
                         input_qos.get_rmw_qos_profile());
    mask_sub_.subscribe(this, config_.mask_topic,
                        input_qos.get_rmw_qos_profile());
    sync_ = std::make_unique<Sync>(
        SyncPolicy(static_cast<uint32_t>(config_.sync_queue_size)), depth_sub_,
        mask_sub_);
    sync_->registerCallback(std::bind(&TsdfOnlyNode::handleImages, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        config_.tf_topic, input_qos,
        std::bind(&TsdfOnlyNode::handleTf, this, std::placeholders::_1));

    tsdf_pub_ = create_publisher<PointCloud2>(
        config_.tsdf_output_topic,
        rclcpp::QoS(static_cast<size_t>(config_.output_queue_size)).reliable());

    const auto publish_period = std::chrono::duration_cast<
        std::chrono::nanoseconds>(
        std::chrono::duration<double>(config_.publish_period_sec));
    publish_timer_ = create_wall_timer(
        publish_period, std::bind(&TsdfOnlyNode::publishTsdfSurface, this));

    RCLCPP_INFO(
        get_logger(),
        "nvblox TSDF-only mapping %s + %s + %s -> %s, frame %s, voxel %.3f m",
        config_.depth_topic.c_str(), config_.mask_topic.c_str(),
        config_.tf_topic.c_str(), config_.tsdf_output_topic.c_str(),
        config_.world_frame.c_str(), config_.voxel_size_m);
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  struct Config {
    std::string depth_topic = "/head_front_left_color_depth";
    std::string mask_topic = "/head_front_left_color_robot_mask";
    std::string tf_topic = "/tf";
    std::string tsdf_output_topic = "/nvblox_tsdf/surface_points";
    std::string world_frame = "world";
    std::string camera_frame = "head_front_left_color";

    int width = 640;
    int height = 480;
    float fx = 211.2f;
    float fy = 211.2f;
    float cx = 291.19999872f;
    float cy = 240.0f;

    float depth_min_m = 0.1f;
    float depth_max_m = 10.0f;
    float depth_scale = 0.001f;
    int mask_robot_threshold = 0;

    float voxel_size_m = 0.05f;
    float truncation_distance_vox = 4.0f;
    float max_weight = 20.0f;
    float max_integration_distance_m = 10.0f;

    float min_visualization_weight = 1.0f;
    float surface_visualization_distance_vox = 1.0f;
    double publish_period_sec = 1.0;

    int input_queue_size = 30;
    int output_queue_size = 1;
    int sync_queue_size = 30;
    int pending_frame_limit = 120;
    double max_image_stamp_delta_sec = 0.002;
    double tf_buffer_duration_sec = 5.0;
    double max_tf_gap_sec = 0.2;
    double max_tf_translation_step_m = 0.5;
    double max_tf_rotation_step_deg = 45.0;
  };

  struct PoseSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int64_t stamp_ns = 0;
    Eigen::Vector3f translation = Eigen::Vector3f::Zero();
    Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
  };

  struct PendingFrame {
    Image::ConstSharedPtr depth;
    Image::ConstSharedPtr mask;
  };

  enum class PoseStatus {
    kReady,
    kWaitingForFuture,
    kDropFrame,
  };

  struct PoseLookup {
    PoseStatus status = PoseStatus::kWaitingForFuture;
    nvblox::Transform pose = nvblox::Transform::Identity();
    std::string reason;
  };

  struct IntegrationStats {
    size_t candidate_pixels = 0;
    size_t masked_pixels = 0;
    size_t range_filtered_pixels = 0;
    size_t active_depth_pixels = 0;
  };

  struct SurfacePoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float distance = 0.0f;
    float weight = 0.0f;
  };

  Config loadConfig() {
    Config config;
    config.depth_topic =
        declare_parameter<std::string>("depth_topic", config.depth_topic);
    config.mask_topic =
        declare_parameter<std::string>("mask_topic", config.mask_topic);
    config.tf_topic = declare_parameter<std::string>("tf_topic", config.tf_topic);
    config.tsdf_output_topic = declare_parameter<std::string>(
        "tsdf_output_topic", config.tsdf_output_topic);
    config.world_frame =
        declare_parameter<std::string>("world_frame", config.world_frame);
    config.camera_frame =
        declare_parameter<std::string>("camera_frame", config.camera_frame);

    config.width = positiveOrDefault(declare_parameter<int>("width", config.width),
                                     config.width);
    config.height = positiveOrDefault(
        declare_parameter<int>("height", config.height), config.height);
    config.fx = positiveOrDefault(
        declare_parameter<double>("fx", config.fx), config.fx);
    config.fy = positiveOrDefault(
        declare_parameter<double>("fy", config.fy), config.fy);
    config.cx = static_cast<float>(declare_parameter<double>("cx", config.cx));
    config.cy = static_cast<float>(declare_parameter<double>("cy", config.cy));

    config.depth_min_m = static_cast<float>(
        declare_parameter<double>("depth_min_m", config.depth_min_m));
    config.depth_max_m = static_cast<float>(
        declare_parameter<double>("depth_max_m", config.depth_max_m));
    config.depth_scale = positiveOrDefault(
        declare_parameter<double>("depth_scale", config.depth_scale),
        config.depth_scale);
    const int64_t mask_robot_threshold =
        declare_parameter<int>("mask_robot_threshold",
                               config.mask_robot_threshold);
    config.mask_robot_threshold =
        static_cast<int>(std::clamp<int64_t>(mask_robot_threshold, 0, 255));

    config.voxel_size_m = positiveOrDefault(
        declare_parameter<double>("voxel_size_m", config.voxel_size_m),
        config.voxel_size_m);
    config.truncation_distance_vox = positiveOrDefault(
        declare_parameter<double>("truncation_distance_vox",
                                  config.truncation_distance_vox),
        config.truncation_distance_vox);
    config.max_weight = positiveOrDefault(
        declare_parameter<double>("max_weight", config.max_weight),
        config.max_weight);
    config.max_integration_distance_m = positiveOrDefault(
        declare_parameter<double>("max_integration_distance_m",
                                  config.max_integration_distance_m),
        config.max_integration_distance_m);

    config.min_visualization_weight = positiveOrDefault(
        declare_parameter<double>("min_visualization_weight",
                                  config.min_visualization_weight),
        config.min_visualization_weight);
    config.surface_visualization_distance_vox = positiveOrDefault(
        declare_parameter<double>("surface_visualization_distance_vox",
                                  config.surface_visualization_distance_vox),
        config.surface_visualization_distance_vox);
    config.publish_period_sec = positiveOrDefault(
        declare_parameter<double>("publish_period_sec",
                                  config.publish_period_sec),
        config.publish_period_sec);

    config.input_queue_size = positiveOrDefault(
        declare_parameter<int>("input_queue_size", config.input_queue_size), 1);
    config.output_queue_size = positiveOrDefault(
        declare_parameter<int>("output_queue_size", config.output_queue_size), 1);
    config.sync_queue_size = positiveOrDefault(
        declare_parameter<int>("sync_queue_size", config.sync_queue_size), 1);
    config.pending_frame_limit = positiveOrDefault(
        declare_parameter<int>("pending_frame_limit", config.pending_frame_limit),
        1);
    config.max_image_stamp_delta_sec = positiveOrDefault(
        declare_parameter<double>("max_image_stamp_delta_sec",
                                  config.max_image_stamp_delta_sec),
        config.max_image_stamp_delta_sec);
    config.tf_buffer_duration_sec = positiveOrDefault(
        declare_parameter<double>("tf_buffer_duration_sec",
                                  config.tf_buffer_duration_sec),
        config.tf_buffer_duration_sec);
    config.max_tf_gap_sec = positiveOrDefault(
        declare_parameter<double>("max_tf_gap_sec", config.max_tf_gap_sec),
        config.max_tf_gap_sec);
    config.max_tf_translation_step_m = declare_parameter<double>(
        "max_tf_translation_step_m", config.max_tf_translation_step_m);
    config.max_tf_rotation_step_deg = declare_parameter<double>(
        "max_tf_rotation_step_deg", config.max_tf_rotation_step_deg);
    return config;
  }

  template <typename T, typename U>
  T positiveOrDefault(const U value, const T default_value) const {
    const T converted_value = static_cast<T>(value);
    return converted_value > T(0) ? converted_value : default_value;
  }

  void handleImages(const Image::ConstSharedPtr& depth_msg,
                    const Image::ConstSharedPtr& mask_msg) {
    if (!validateImages(*depth_msg, *mask_msg) ||
        !validateImageStamps(*depth_msg, *mask_msg)) {
      return;
    }

    pending_frames_.push_back(PendingFrame{depth_msg, mask_msg});
    while (static_cast<int>(pending_frames_.size()) >
           config_.pending_frame_limit) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "dropping oldest pending RGB-D frame because pending queue exceeded %d",
          config_.pending_frame_limit);
      pending_frames_.pop_front();
    }
    processPendingFrames();
  }

  void handleTf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    bool added_sample = false;
    for (const auto& tf : msg->transforms) {
      if (!frameMatches(tf.header.frame_id, config_.world_frame) ||
          !frameMatches(tf.child_frame_id, config_.camera_frame)) {
        continue;
      }

      PoseSample sample;
      sample.stamp_ns = stampToNanoseconds(tf.header.stamp);
      sample.translation = Eigen::Vector3f(
          static_cast<float>(tf.transform.translation.x),
          static_cast<float>(tf.transform.translation.y),
          static_cast<float>(tf.transform.translation.z));
      sample.rotation = Eigen::Quaternionf(
          static_cast<float>(tf.transform.rotation.w),
          static_cast<float>(tf.transform.rotation.x),
          static_cast<float>(tf.transform.rotation.y),
          static_cast<float>(tf.transform.rotation.z));
      sample.rotation.normalize();
      insertPoseSample(sample);
      added_sample = true;
    }

    if (added_sample) {
      prunePoseSamples();
      processPendingFrames();
    }
  }

  bool validateImages(const Image& depth, const Image& mask) {
    if (depth.data.empty() || mask.data.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "received an empty depth or mask image");
      return false;
    }
    if (!isDepthEncoding(depth.encoding)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "depth image must use 32FC1 meters or 16UC1 scaled encoding, got '%s'",
          depth.encoding.c_str());
      return false;
    }
    if (!isMaskEncoding(mask.encoding)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "mask image must use mono8 or 8UC1 encoding, got '%s'",
          mask.encoding.c_str());
      return false;
    }
    if (depth.is_bigendian || mask.is_bigendian) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "big-endian image buffers are not supported");
      return false;
    }
    if (depth.height != mask.height || depth.width != mask.width) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "depth and mask image sizes do not match");
      return false;
    }

    const size_t depth_pixel_size =
        depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1
            ? sizeof(float)
            : sizeof(uint16_t);
    const size_t depth_min_step =
        static_cast<size_t>(depth.width) * depth_pixel_size;
    const size_t mask_min_step = static_cast<size_t>(mask.width);
    if (depth.step < depth_min_step || mask.step < mask_min_step ||
        depth.data.size() < static_cast<size_t>(depth.step) * depth.height ||
        mask.data.size() < static_cast<size_t>(mask.step) * mask.height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "depth or mask image buffer is too small");
      return false;
    }
    return true;
  }

  bool validateImageStamps(const Image& depth, const Image& mask) {
    const double delta = stampDeltaSec(depth.header.stamp, mask.header.stamp);
    if (delta <= config_.max_image_stamp_delta_sec) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping depth/mask frame with stamp mismatch %.6fs, limit %.6fs",
        delta, config_.max_image_stamp_delta_sec);
    return false;
  }

  void processPendingFrames() {
    while (!pending_frames_.empty()) {
      const PendingFrame frame = pending_frames_.front();
      const PoseLookup lookup = lookupInterpolatedPose(frame.depth->header.stamp);
      if (lookup.status == PoseStatus::kWaitingForFuture) {
        return;
      }

      pending_frames_.pop_front();
      if (lookup.status == PoseStatus::kDropFrame) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "dropping depth frame at %.9f: %s",
            static_cast<double>(stampToNanoseconds(frame.depth->header.stamp)) *
                1.0e-9,
            lookup.reason.c_str());
        continue;
      }
      integrateFrameWithPose(*frame.depth, *frame.mask, lookup.pose);
    }
  }

  void integrateFrameWithPose(const Image& depth, const Image& mask,
                              const nvblox::Transform& T_L_C) {
    const IntegrationStats stats = fillNvbloxImages(depth, mask);

    const nvblox::MonoImageConstView mask_view(mask_image_);
    const nvblox::MaskedDepthImageConstView depth_view(
        depth_image_,
        std::optional<nvblox::ImageView<const uint8_t>>(mask_view),
        nvblox::MaskMode::kInverted);
    mapper_->integrateDepth(depth_view, T_L_C, camera_);

    last_stamp_ = depth.header.stamp;
    have_input_ = true;
    ++integrated_frames_;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "integrated frame %zu: active_depth=%zu/%zu, masked=%zu, range=%zu, "
        "tsdf_blocks=%zu",
        integrated_frames_, stats.active_depth_pixels, stats.candidate_pixels,
        stats.masked_pixels, stats.range_filtered_pixels,
        static_cast<size_t>(mapper_->tsdf_layer().numBlocks()));
  }

  IntegrationStats fillNvbloxImages(const Image& depth, const Image& mask) {
    if (depth_image_.rows() != static_cast<int>(depth.height) ||
        depth_image_.cols() != static_cast<int>(depth.width)) {
      depth_image_.resizeAsync(depth.height, depth.width, cuda_stream_);
      mask_image_.resizeAsync(mask.height, mask.width, cuda_stream_);
      cuda_stream_.synchronize();
    }

    IntegrationStats stats;
    const bool depth_is_float =
        depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1;
    const size_t depth_pixel_size =
        depth_is_float ? sizeof(float) : sizeof(uint16_t);

    for (uint32_t v = 0; v < depth.height; ++v) {
      for (uint32_t u = 0; u < depth.width; ++u) {
        ++stats.candidate_pixels;

        const uint8_t mask_value =
            mask.data[static_cast<size_t>(v) * mask.step + u];
        const bool is_robot = mask_value > config_.mask_robot_threshold;
        mask_image_(v, u) = is_robot ? 1 : 0;
        if (is_robot) {
          ++stats.masked_pixels;
        }

        const uint8_t* depth_ptr =
            depth.data.data() + static_cast<size_t>(v) * depth.step +
            static_cast<size_t>(u) * depth_pixel_size;
        float depth_m =
            depth_is_float
                ? readUnaligned<float>(depth_ptr)
                : static_cast<float>(readUnaligned<uint16_t>(depth_ptr)) *
                      config_.depth_scale;
        if (!std::isfinite(depth_m) || depth_m < config_.depth_min_m ||
            depth_m > config_.depth_max_m) {
          depth_m = 0.0f;
          ++stats.range_filtered_pixels;
        } else {
          ++stats.active_depth_pixels;
        }
        depth_image_(v, u) = depth_m;
      }
    }
    return stats;
  }

  void publishTsdfSurface() {
    if (!have_input_ && mapper_->tsdf_layer().numBlocks() == 0) {
      return;
    }

    std::vector<SurfacePoint> points;
    collectSurfacePoints(&points);

    PointCloud2 cloud;
    cloud.header.frame_id = config_.world_frame;
    if (have_input_) {
      cloud.header.stamp = last_stamp_;
    } else {
      cloud.header.stamp = now();
    }
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1,
        sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
        sensor_msgs::msg::PointField::FLOAT32, "weight", 1,
        sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity_it(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> weight_it(cloud, "weight");
    for (const SurfacePoint& point : points) {
      *x_it = point.x;
      *y_it = point.y;
      *z_it = point.z;
      *intensity_it = point.distance;
      *weight_it = point.weight;
      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
      ++weight_it;
    }

    tsdf_pub_->publish(cloud);
  }

  void collectSurfacePoints(std::vector<SurfacePoint>* points) const {
    CHECK_NOTNULL(points);
    const nvblox::TsdfLayer& layer = mapper_->tsdf_layer();
    const float block_size = layer.block_size();
    const float voxel_size = layer.voxel_size();
    const float max_surface_distance =
        config_.surface_visualization_distance_vox * voxel_size;
    constexpr int kVoxelsPerSide = nvblox::VoxelBlock<nvblox::TsdfVoxel>::kVoxelsPerSide;

    const std::vector<nvblox::Index3D> block_indices =
        layer.getAllBlockIndices();
    points->reserve(block_indices.size() * 32);
    for (const nvblox::Index3D& block_index : block_indices) {
      const nvblox::TsdfBlock::ConstPtr block =
          layer.getBlockAtIndex(block_index);
      if (block == nullptr) {
        continue;
      }
      for (int x = 0; x < kVoxelsPerSide; ++x) {
        for (int y = 0; y < kVoxelsPerSide; ++y) {
          for (int z = 0; z < kVoxelsPerSide; ++z) {
            const nvblox::Index3D voxel_index(x, y, z);
            const nvblox::TsdfVoxel& voxel = (*block)(voxel_index);
            if (voxel.weight < config_.min_visualization_weight ||
                std::abs(voxel.distance) > max_surface_distance) {
              continue;
            }
            const nvblox::Vector3f position =
                nvblox::getCenterPositionFromBlockIndexAndVoxelIndex(
                    block_size, block_index, voxel_index);
            points->push_back(SurfacePoint{
                position.x(), position.y(), position.z(), voxel.distance,
                voxel.weight});
          }
        }
      }
    }
  }

  void insertPoseSample(const PoseSample& sample) {
    auto it = std::lower_bound(
        pose_samples_.begin(), pose_samples_.end(), sample.stamp_ns,
        [](const PoseSample& lhs, int64_t stamp_ns) {
          return lhs.stamp_ns < stamp_ns;
        });
    if (it != pose_samples_.end() && it->stamp_ns == sample.stamp_ns) {
      *it = sample;
      return;
    }
    pose_samples_.insert(it, sample);
  }

  void prunePoseSamples() {
    if (pose_samples_.size() <= 2) {
      return;
    }
    const int64_t min_stamp =
        pose_samples_.back().stamp_ns -
        secondsToNanoseconds(config_.tf_buffer_duration_sec);
    while (pose_samples_.size() > 2 && pose_samples_[1].stamp_ns < min_stamp) {
      pose_samples_.erase(pose_samples_.begin());
    }
  }

  nvblox::Transform sampleToPose(const PoseSample& sample) const {
    nvblox::Transform pose = nvblox::Transform::Identity();
    pose.linear() = sample.rotation.toRotationMatrix();
    pose.translation() = sample.translation;
    return pose;
  }

  bool isPoseStepContinuous(const PoseSample& lhs,
                            const PoseSample& rhs) const {
    if (config_.max_tf_translation_step_m > 0.0) {
      const float translation_step = (rhs.translation - lhs.translation).norm();
      if (translation_step >
          static_cast<float>(config_.max_tf_translation_step_m)) {
        return false;
      }
    }

    if (config_.max_tf_rotation_step_deg > 0.0) {
      constexpr double kPi = 3.14159265358979323846;
      const double max_rotation_step_rad =
          config_.max_tf_rotation_step_deg * kPi / 180.0;
      if (lhs.rotation.angularDistance(rhs.rotation) > max_rotation_step_rad) {
        return false;
      }
    }
    return true;
  }

  bool isPoseSampleContinuous(size_t index) const {
    if (index >= pose_samples_.size()) {
      return false;
    }
    if (index > 0 &&
        !isPoseStepContinuous(pose_samples_[index - 1],
                              pose_samples_[index])) {
      return false;
    }
    if (index + 1 < pose_samples_.size() &&
        !isPoseStepContinuous(pose_samples_[index],
                              pose_samples_[index + 1])) {
      return false;
    }
    return true;
  }

  PoseLookup lookupInterpolatedPose(
      const builtin_interfaces::msg::Time& stamp_msg) const {
    PoseLookup result;
    if (pose_samples_.empty()) {
      result.reason = "no direct TF samples buffered yet";
      return result;
    }

    const int64_t target_ns = stampToNanoseconds(stamp_msg);
    constexpr int64_t kExactToleranceNs = 1000;
    auto next_it = std::lower_bound(
        pose_samples_.begin(), pose_samples_.end(), target_ns,
        [](const PoseSample& lhs, int64_t stamp_ns) {
          return lhs.stamp_ns < stamp_ns;
        });

    if (next_it != pose_samples_.end() &&
        std::abs(next_it->stamp_ns - target_ns) <= kExactToleranceNs) {
      const auto index =
          static_cast<size_t>(std::distance(pose_samples_.begin(), next_it));
      if (!isPoseSampleContinuous(index)) {
        result.status = PoseStatus::kDropFrame;
        result.reason = "direct TF exact sample is adjacent to a pose jump";
        return result;
      }
      result.status = PoseStatus::kReady;
      result.pose = sampleToPose(*next_it);
      return result;
    }

    if (next_it == pose_samples_.begin()) {
      result.status = PoseStatus::kDropFrame;
      result.reason = "image stamp is older than the first buffered direct TF sample";
      return result;
    }
    if (next_it == pose_samples_.end()) {
      result.status = PoseStatus::kWaitingForFuture;
      result.reason = "waiting for a newer direct TF sample for interpolation";
      return result;
    }

    const auto prev_it = std::prev(next_it);
    const int64_t gap_ns = next_it->stamp_ns - prev_it->stamp_ns;
    const int64_t max_gap_ns = secondsToNanoseconds(config_.max_tf_gap_sec);
    if (gap_ns <= 0 || gap_ns > max_gap_ns) {
      result.status = PoseStatus::kDropFrame;
      result.reason = "direct TF interpolation gap is too large";
      return result;
    }
    if (!isPoseStepContinuous(*prev_it, *next_it)) {
      result.status = PoseStatus::kDropFrame;
      result.reason = "direct TF interpolation segment has a pose jump";
      return result;
    }

    const float ratio =
        static_cast<float>(static_cast<double>(target_ns - prev_it->stamp_ns) /
                           static_cast<double>(gap_ns));
    Eigen::Quaternionf rotation = prev_it->rotation.slerp(ratio, next_it->rotation);
    rotation.normalize();

    result.status = PoseStatus::kReady;
    result.pose = nvblox::Transform::Identity();
    result.pose.linear() = rotation.toRotationMatrix();
    result.pose.translation() =
        prev_it->translation + ratio * (next_it->translation - prev_it->translation);
    return result;
  }

  Config config_;
  std::unique_ptr<nvblox::Mapper> mapper_;
  nvblox::Camera camera_;
  nvblox::CudaStreamOwning cuda_stream_;
  nvblox::DepthImage depth_image_{nvblox::MemoryType::kUnified};
  nvblox::MonoImage mask_image_{nvblox::MemoryType::kUnified};

  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<Image> mask_sub_;
  std::unique_ptr<Sync> sync_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr tsdf_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::vector<PoseSample, Eigen::aligned_allocator<PoseSample>> pose_samples_;
  std::deque<PendingFrame> pending_frames_;

  builtin_interfaces::msg::Time last_stamp_;
  bool have_input_ = false;
  size_t integrated_frames_ = 0;
};

}  // namespace nvblox_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<nvblox_ros::TsdfOnlyNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
