#include <esim/visualization/text_file_publisher.hpp>
#include <esim/common/utils.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>

DEFINE_string(path_to_events_text_file, "",
"Path to the text file in which to write the events.");

namespace event_camera_simulator {

TextFilePublisher::TextFilePublisher(const std::string& path_to_events_text_file)
{
  events_text_file_.open(path_to_events_text_file);
  events_text_file_ << "t,x,y,p,last_camera_pose:t,x,y,z,wx,wy,wz,ww" << std::endl;
  last_pose_ = geometry_msgs::PoseStamped();
  last_pose_t_ = 0;
}

TextFilePublisher::~TextFilePublisher()
{
    events_text_file_.close();
}

Publisher::Ptr TextFilePublisher::createFromGflags(const std::string& path_to_events_text_file)
{
  if(path_to_events_text_file == "")
  {
    LOG(WARNING) << "Empty events text file path: will not write events to a text file.";
    return nullptr;
  }

  return std::make_shared<TextFilePublisher>(path_to_events_text_file);
}

void TextFilePublisher::eventsCallback(const EventsVector& events)
{
    CHECK_EQ(events.size(), 1);
    for(const Event& e : events[0])
    {
        events_text_file_ << e.t << "," << e.x << "," << e.y << "," << e.pol << ",";
	events_text_file_ << last_pose_t_ << "," << last_pose_.pose.position.x << "," << last_pose_.pose.position.y << "," <<  last_pose_.pose.position.z << ",";
        events_text_file_ << last_pose_.pose.orientation.x << "," << last_pose_.pose.orientation.y << "," << last_pose_.pose.orientation.z << "," << last_pose_.pose.orientation.w << std::endl;
    }

}

void TextFilePublisher::poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t)
{
    //LOG(INFO) << "t: " << t;
    last_pose_t_ = t;
    tf::poseStampedKindrToMsg(T_W_Cs[0], toRosTime(t), "camera", &last_pose_);
    //LOG(INFO) << "msg t: " << last_pose_.header.stamp.nsec;

    //std::cout << last_pose_t_ << "|" << last_pose_ << std::endl;
}

} // namespace event_camera_simulator
