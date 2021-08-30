#pragma once

#include <esim/common/types.hpp>
#include <esim/visualization/ros_utils.hpp>
#include <esim/visualization/publisher_interface.hpp>
#include <fstream>
#include <tf/tfMessage.h>
#include <minkindr_conversions/kindr_msg.h>

#include <algorithm> 
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <random>
#include <vector> 
#include <sys/stat.h>

namespace event_camera_simulator {

class TextFilePublisher : public Publisher
{
public:
  TextFilePublisher(const std::string& corner_filename, const std::string& output_folder);
  ~TextFilePublisher();

  static Publisher::Ptr createFromGflags(const std::string& corner_filename, const std::string& output_folder);

  virtual void cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t) override;
  virtual bool eventsCallback(const EventsVector& events) override; // will be called when new events are available
  virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t) override;
  virtual void rendererInfoCallback(const Transformation& T_W_P, const CalibrationMatrix& K_inv, Time t) override;

  virtual void imageCallback(const ImagePtrVector& images, Time t) override {}
  virtual void imageCorruptedCallback(const ImagePtrVector& corrupted_images, Time t) override {}
  virtual void depthmapCallback(const DepthmapPtrVector& depthmaps, Time t) override {}
  virtual void opticFlowCallback(const OpticFlowPtrVector& optic_flows, Time t) override {}
  virtual void twistCallback(const AngularVelocityVector& ws, const LinearVelocityVector& vs, Time t) override {}
  virtual void imuCallback(const Vector3& acc, const Vector3& gyr, Time t) override {}
  virtual void pointcloudCallback(const PointCloudVector& pointclouds, Time t) override {}

private:
  std::vector<std::ofstream> events_text_files_;
  std::vector<int> event_counters_;
  geometry_msgs::PoseStamped last_pose_;
  uint64_t last_pose_t_;

  int num_corners_;
  ze::Bearings corners_P_;
  ze::Bearings bearings_P_;
  Transformation T_W_P_;
  ze::Keypoints corners_C_;
  ze::Camera::Ptr dvs_camera_;
};

} // namespace event_camera_simulator
