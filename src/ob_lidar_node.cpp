/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "orbbec_camera/ob_lidar_node.h"
#include "libobsensor/hpp/Utils.hpp"
#if defined(USE_RK_HW_DECODER)
#include "orbbec_camera/rk_mpp_decoder.h"
#elif defined(USE_NV_HW_DECODER)
#include "orbbec_camera/jetson_nv_decoder.h"
#endif

namespace orbbec_camera {
namespace orbbec_lidar {
OBLidarNode::OBLidarNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                         std::shared_ptr<ob::Device> device)
    : nh_(nh),
      nh_private_(nh_private),
      device_(std::move(device)),
      device_info_(device_->getDeviceInfo()) {
  stream_name_[LIDAR] = "lidar";
  init();
}

void OBLidarNode::init() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  CHECK_NOTNULL(device_.get());
  is_running_ = true;
  getParameters();
  ROS_INFO_STREAM("JJJJJJJJJ");
//   setupDevices();
//   selectBaseStream();
//   setupProfiles();
//   setupTopics();
//   setupFrameCallback();
#if defined(USE_RK_HW_DECODER)
  mjpeg_decoder_ = std::make_shared<RKMjpegDecoder>(width_[COLOR], height_[COLOR]);
#elif defined(USE_NV_HW_DECODER)
  mjpeg_decoder_ = std::make_shared<JetsonNvJPEGDecoder>(width_[COLOR], height_[COLOR]);
#endif
  is_initialized_ = true;
}

bool OBLidarNode::isInitialized() const { return is_initialized_; }

void OBLidarNode::rebootDevice() {
  ROS_INFO("Reboot device");
  if (device_) {
    device_->reboot();
  }
  ROS_INFO("Reboot device DONE");
}

void OBLidarNode::clean() {
  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() start");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  is_running_ = false;
  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() stop tf thread");
  if (tf_thread_ && tf_thread_->joinable()) {
    tf_thread_->join();
  }

  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() stop stream");
//   stopStreams();
  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() delete rgb_buffer");
  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() end");
}

OBLidarNode::~OBLidarNode() noexcept { clean(); }

void OBLidarNode::getParameters() {
  camera_name_ = nh_private_.param<std::string>("camera_name", "lidar");
  for (const auto& stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_format";
    format_str_[stream_index] =
        nh_private_.param<std::string>(param_name, format_str_[stream_index]);
    format_[stream_index] = OBFormatFromString(format_str_[stream_index]);
    param_name = stream_name_[stream_index] + "_rate";
    rate_int_[stream_index] = nh_private_.param<int>(param_name, rate_int_[stream_index]);
    rate_[stream_index] = OBScanRateFromInt(rate_int_[stream_index]);
  }
}

// void OBLidarNode::startStreams() {
//   std::lock_guard<decltype(device_lock_)> lock(device_lock_);
//   if (enable_pipeline_) {
//     CHECK_NOTNULL(pipeline_.get());
//     if (enable_frame_sync_) {
//       ROS_INFO_STREAM("====Enable frame sync====");
//       pipeline_->enableFrameSync();
//     } else {
//       pipeline_->disableFrameSync();
//     }
//     try {
//       setupPipelineConfig();
//       pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet>& frame_set) {
//         CHECK_NOTNULL(frame_set.get());
//         this->onNewFrameSetCallback(frame_set);
//       });
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("failed to start pipeline: " << e.getMessage()
//                                                     << " try to disable ir stream try again");
//       enable_stream_[INFRA0] = false;
//       setupPipelineConfig();
//       pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet>& frame_set) {
//         CHECK_NOTNULL(frame_set.get());
//         this->onNewFrameSetCallback(frame_set);
//       });
//     } catch (...) {
//       ROS_ERROR_STREAM("failed to start pipeline");
//       throw;
//     }
//     init_interleave_mode();
//     if (!colorFrameThread_ && enable_stream_[COLOR]) {
//       ROS_INFO_STREAM("Create color frame read thread.");
//       colorFrameThread_ = std::make_shared<std::thread>([this]() { onNewColorFrameCallback(); });
//     }
//     pipeline_started_ = true;
//   } else {
//     for (const auto& stream_index : IMAGE_STREAMS) {
//       if (enable_stream_[stream_index] && !stream_started_[stream_index]) {
//         startStream(stream_index);
//       }
//     }
//   }
// }

// void OBLidarNode::stopStreams() {
//   std::lock_guard<decltype(device_lock_)> lock(device_lock_);
//   if (enable_pipeline_ && pipeline_ && pipeline_started_) {
//     CHECK_NOTNULL(pipeline_.get());
//     try {
//       pipeline_->stop();
//       // disable interleave frame
//       if ((interleave_ae_mode_ == "hdr") || (interleave_ae_mode_ == "laser") && !is_running_) {
//         ROS_INFO_STREAM("current interleave_ae_mode_: " << interleave_ae_mode_);
//         if (device_->isPropertySupported(OB_PROP_FRAME_INTERLEAVE_ENABLE_BOOL,
//                                          OB_PERMISSION_WRITE)) {
//           interleave_frame_enable_ = false;
//           ROS_INFO_STREAM("Enable enable_interleave_depth_frame to "
//                           << (interleave_frame_enable_ ? "true" : "false"));
//           device_->setBoolProperty(OB_PROP_FRAME_INTERLEAVE_ENABLE_BOOL,
//           interleave_frame_enable_);
//         }
//       }
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("Failed to stop pipeline: " << e.getMessage());
//     }
//     pipeline_started_ = false;
//   } else {
//     for (const auto& stream_index : IMAGE_STREAMS) {
//       if (stream_started_[stream_index]) {
//         stopStream(stream_index);
//       }
//     }
//   }
// }

// void OBLidarNode::publishStaticTF(const ros::Time& t, const tf2::Vector3& trans,
//                                    const tf2::Quaternion& q, const std::string& from,
//                                    const std::string& to) {
//   geometry_msgs::TransformStamped msg;
//   msg.header.stamp = t;
//   msg.header.frame_id = from;
//   msg.child_frame_id = to;
//   msg.transform.translation.x = trans[2] / 1000.0;
//   msg.transform.translation.y = -trans[0] / 1000.0;
//   msg.transform.translation.z = -trans[1] / 1000.0;
//   msg.transform.rotation.x = q.getX();
//   msg.transform.rotation.y = q.getY();
//   msg.transform.rotation.z = q.getZ();
//   msg.transform.rotation.w = q.getW();
//   static_tf_msgs_.push_back(msg);
// }

// void OBLidarNode::calcAndPublishStaticTransform() {
//   tf2::Quaternion quaternion_optical, zero_rot;
//   zero_rot.setRPY(0.0, 0.0, 0.0);
//   quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
//   tf2::Vector3 zero_trans(0, 0, 0);
//   if (!stream_profile_.count(base_stream_)) {
//     ROS_ERROR_STREAM("Base stream is not available");
//     return;
//   }
//   auto base_stream_profile = stream_profile_[base_stream_];
//   CHECK_NOTNULL(base_stream_profile.get());
//   for (const auto& item : stream_profile_) {
//     auto stream_index = item.first;
//     auto stream_profile = item.second;
//     if (!stream_profile) {
//       continue;
//     }
//     OBExtrinsic ex;
//     try {
//       ex = stream_profile->getExtrinsicTo(base_stream_profile);
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("Failed to get " << stream_name_[stream_index]
//                                         << " extrinsic: " << e.getMessage());
//       ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
//     }

//     auto Q = rotationMatrixToQuaternion(ex.rot);
//     Q = quaternion_optical * Q * quaternion_optical.inverse();
//     Q = Q.normalize();
//     tf2::Vector3 trans(ex.trans[0], ex.trans[1], ex.trans[2]);

//     auto timestamp = ros::Time::now();
//     if (stream_index.first != base_stream_.first) {
//       if (stream_index.first == OB_STREAM_IR_RIGHT && base_stream_.first == OB_STREAM_DEPTH) {
//         trans[0] = std::abs(trans[0]);  // because left and right ir calibration is error
//       }
//       publishStaticTF(timestamp, trans, Q, frame_id_[base_stream_], frame_id_[stream_index]);
//     }
//     publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
//                     optical_frame_id_[stream_index]);
//     ROS_INFO_STREAM("Publishing static transform from " << stream_name_[stream_index] << " to "
//                                                         << stream_name_[base_stream_]);
//     ROS_INFO_STREAM("Translation " << trans[0] << ", " << trans[1] << ", " << trans[2]);
//     ROS_INFO_STREAM("Rotation " << Q.getX() << ", " << Q.getY() << ", " << Q.getZ() << ", "
//                                 << Q.getW());
//   }
//   auto device_info = device_->getDeviceInfo();
//   CHECK_NOTNULL(device_info);
//   auto pid = device_info->pid();
//   if ((pid == FEMTO_BOLT_PID || pid == FEMTO_MEGA_PID) && enable_stream_[DEPTH] &&
//       enable_stream_[COLOR]) {
//     // calc depth to color
//     CHECK_NOTNULL(stream_profile_[COLOR]);
//     auto depth_to_color_extrinsics = base_stream_profile->getExtrinsicTo(stream_profile_[COLOR]);
//     auto Q = rotationMatrixToQuaternion(depth_to_color_extrinsics.rot);
//     Q = quaternion_optical * Q * quaternion_optical.inverse();
//     Q = Q.normalize();
//     publishStaticTF(ros::Time::now(), zero_trans, Q, camera_link_frame_id_,
//                     frame_id_[base_stream_]);
//   } else {
//     publishStaticTF(ros::Time::now(), zero_trans, zero_rot, camera_link_frame_id_,
//                     frame_id_[base_stream_]);
//   }
// }

// void OBLidarNode::publishStaticTransforms() {
//   static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
//   dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
//   auto base_stream_profile = stream_profile_[base_stream_];
//   if (enable_stream_[DEPTH] && enable_stream_[COLOR]) {
//     static const char* frame_id = "depth_to_color_extrinsics";
//     OBExtrinsic ex;
//     try {
//       ex = base_stream_profile->getExtrinsicTo(stream_profile_[COLOR]);
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
//       ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
//     }
//     depth_to_other_extrinsics_[COLOR] = ex;
//     auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
//     depth_to_other_extrinsics_publishers_[COLOR].publish(ex_msg);
//   }
//   if (enable_stream_[DEPTH] && enable_stream_[INFRA0]) {
//     static const char* frame_id = "depth_to_ir_extrinsics";
//     OBExtrinsic ex;
//     try {
//       ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA0]);
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
//       ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
//     }
//     depth_to_other_extrinsics_[INFRA0] = ex;
//     auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
//     depth_to_other_extrinsics_publishers_[INFRA0].publish(ex_msg);
//   }
//   if (enable_stream_[DEPTH] && enable_stream_[INFRA1]) {
//     static const char* frame_id = "depth_to_left_ir_extrinsics";
//     OBExtrinsic ex;
//     try {
//       ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA1]);
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
//       ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
//     }
//     depth_to_other_extrinsics_[INFRA1] = ex;
//     auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
//     depth_to_other_extrinsics_publishers_[INFRA1].publish(ex_msg);
//   }
//   if (enable_stream_[DEPTH] && enable_stream_[INFRA2]) {
//     static const char* frame_id = "depth_to_right_ir_extrinsics";
//     OBExtrinsic ex;
//     try {
//       ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA2]);
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
//       ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
//     }
//     depth_to_other_extrinsics_[INFRA2] = ex;
//     auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
//     depth_to_other_extrinsics_publishers_[INFRA2].publish(ex_msg);
//   }
//   if (enable_stream_[DEPTH] && enable_stream_[ACCEL]) {
//     static const char* frame_id = "depth_to_accel_extrinsics";
//     OBExtrinsic ex;
//     try {
//       ex = base_stream_profile->getExtrinsicTo(stream_profile_[ACCEL]);
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
//       ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
//     }
//     depth_to_other_extrinsics_[ACCEL] = ex;
//     auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
//     depth_to_other_extrinsics_publishers_[ACCEL].publish(ex_msg);
//   }
//   if (enable_stream_[DEPTH] && enable_stream_[GYRO]) {
//     static const char* frame_id = "depth_to_gyro_extrinsics";
//     OBExtrinsic ex;
//     try {
//       ex = base_stream_profile->getExtrinsicTo(stream_profile_[GYRO]);
//     } catch (const ob::Error& e) {
//       ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
//       ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
//     }
//     depth_to_other_extrinsics_[GYRO] = ex;
//     auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
//     depth_to_other_extrinsics_publishers_[GYRO].publish(ex_msg);
//   }
//   calcAndPublishStaticTransform();
//   if (tf_publish_rate_ > 0) {
//     tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
//   } else {
//     CHECK_NOTNULL(static_tf_broadcaster_.get());
//     static_tf_broadcaster_->sendTransform(static_tf_msgs_);
//   }
// }

}  // namespace orbbec_lidar
}  // namespace orbbec_camera
