#include <esim/visualization/text_file_publisher.hpp>
#include <esim/common/utils.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace event_camera_simulator {

TextFilePublisher::TextFilePublisher(const std::string& corner_filename, const std::string& output_filename)
{
  events_text_file_.open(output_filename);
  // events_text_file_ << "t,x,y,p,last_camera_pose:t,x,y,z,wx,wy,wz,ww" << std::endl;
  events_text_file_ << "t,x,y,p,min corner distance, min corner index" << std::endl;
  last_pose_ = geometry_msgs::PoseStamped();
  last_pose_t_ = 0;

  std::ifstream corner_file;
  corner_file.open(corner_filename.c_str());
  std::string corner_line;
  if (!corner_file.is_open())
  {
    LOG(FATAL) << "Couldn't open corner file at:" << corner_filename;
  }
  std::getline(corner_file, corner_line);
  num_corners_ = std::stoi(corner_line);
  if (num_corners_ > 0)
  {
    corners_P_.resize(3, num_corners_);
    bearings_P_.resize(3, num_corners_);
    corners_C_.resize(2, num_corners_);
    std::string x_str, y_str;
    for (int i=0; i<num_corners_; ++i)
    {
      std::getline(corner_file, corner_line);
      std::stringstream ss(corner_line);
      ss >> x_str >> y_str;
      corners_P_.col(i) = ze::Bearing(std::stof(x_str), std::stof(y_str), 1.0);
    }
  }
}

TextFilePublisher::~TextFilePublisher()
{
    events_text_file_.close();
}

Publisher::Ptr TextFilePublisher::createFromGflags(const std::string& corner_filename, const std::string& output_filename)
{
  if(output_filename == "")
  {
    LOG(WARNING) << "Empty events text file path: will not write events to a text file.";
    return nullptr;
  }

  return std::make_shared<TextFilePublisher>(corner_filename, output_filename);
}

void TextFilePublisher::cameraInfoCallback(const ze::CameraRig::Ptr& camera_rig, Time t)
{
  if (!dvs_camera_)
  {
    dvs_camera_ = camera_rig->atShared(0);
  }
}

void TextFilePublisher::rendererInfoCallback(const Transformation& T_W_P, const CalibrationMatrix& K_inv, Time t)
{
  if (num_corners_ > 0 && dvs_camera_)
  {
    bearings_P_ = K_inv * corners_P_;
    bearings_P_.array().rowwise() /= bearings_P_.row(2).array();
    T_W_P_ = T_W_P;
    // LOG(WARNING) << "bearings_P_\n" << bearings_P_;
  }
}

void TextFilePublisher::poseCallback(const Transformation& T_W_B, const TransformationVector& T_W_Cs, Time t)
{
  if (num_corners_ > 0)
  {
    // last_pose_t_ = t;
    // tf::poseStampedKindrToMsg(T_W_Cs[0], toRosTime(t), "camera", &last_pose_);
    // LOG(WARNING) << "T_W_C\n" << T_W_Cs[0] << "\nT_W_P_\n" << T_W_P_;
    Transformation T_C_P = T_W_Cs[0].inverse() * T_W_P_;
    // LOG(WARNING) << "T_C_P\n" << T_C_P;
    ze::Matrix33 tmp;
    tmp.col(0) = T_C_P.getRotationMatrix().col(0);
    tmp.col(1) = T_C_P.getRotationMatrix().col(1);
    tmp.col(2) = T_C_P.getPosition();
    HomographyMatrix H_C_P = tmp;
    // LOG(WARNING) << "H_C_P\n" << H_C_P;

    ze::Bearings bearings_C_ = H_C_P * bearings_P_;
    // LOG(WARNING) << "bearings_C_\n" << bearings_C_;
    corners_C_ = dvs_camera_->projectVectorized(bearings_C_);
    // LOG(WARNING) << "corners_C_\n" << corners_C_;
  }
}

void TextFilePublisher::eventsCallback(const EventsVector& events)
{
    CHECK_EQ(events.size(), 1);
    for(const Event& e : events[0])
    {
      events_text_file_ << e.t << "," << e.x << "," << e.y << "," << e.pol;

      if (num_corners_ > 0)
      {
        ze::Keypoint event(e.x, e.y);
        int min_index;
        float min_dist = (corners_C_.colwise() - event).colwise().norm().minCoeff(&min_index);
        events_text_file_ << "," << min_dist << "," << min_index;
        // events_text_file_ << last_pose_t_ << "," << last_pose_.pose.position.x << "," << last_pose_.pose.position.y << "," <<  last_pose_.pose.position.z << ",";
        // events_text_file_ << last_pose_.pose.orientation.x << "," << last_pose_.pose.orientation.y << "," << last_pose_.pose.orientation.z << "," << last_pose_.pose.orientation.w;
      }
      else
      {
        events_text_file_ << "," << -1 << "," << -1;
      }
      
      events_text_file_ << std::endl;
    }

}


} // namespace event_camera_simulator
