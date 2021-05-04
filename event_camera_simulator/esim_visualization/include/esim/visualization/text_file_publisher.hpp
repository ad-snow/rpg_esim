#pragma once

#include <esim/common/types.hpp>
#include <esim/visualization/ros_utils.hpp>
#include <esim/visualization/publisher_interface.hpp>
#include <fstream>
#include <tf/tfMessage.h>
#include <minkindr_conversions/kindr_msg.h>

namespace event_camera_simulator {

class TextFilePublisher : public Publisher
{
public:
  TextFilePublisher(const std::string& path_to_events_text_file, size_t num_cameras);
  ~TextFilePublisher();

  virtual void eventsCallback(const EventsVector& events) override; // will be called when new events are available
   virtual void poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t) override;

  static Publisher::Ptr createFromGflags(size_t num_cameras);


private:
  std::ofstream events_text_file_;
  geometry_msgs::PoseStamped last_pose_;
  uint32_t last_pose_t_;
};

} // namespace event_camera_simulator
