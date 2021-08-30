#include <esim/visualization/text_file_publisher.hpp>
#include <esim/common/utils.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>


DEFINE_int32(num_empty_patches, 10,
             "Number of random patches to track if there are no corners in the scene");

DEFINE_int32(max_num_patches, 10,
             "Number of random patches to track if there are no corners in the scene");

DEFINE_int32(patch_size_half, 32,
             "Number of random patches to track if there are no corners in the scene");

DEFINE_int32(max_events_per_patch, 32000,
             "Number of random patches to track if there are no corners in the scene");

namespace event_camera_simulator {

TextFilePublisher::TextFilePublisher(const std::string& corner_filename, const std::string& output_folder)
{
  last_pose_ = geometry_msgs::PoseStamped();
  last_pose_t_ = 0;

  // Random seed generator
  std::random_device rd;

  // Psuedo random number generator
  std::mt19937 prng(rd());

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
    std::string x_str, y_str;
    std::vector<float> xs(num_corners_), ys(num_corners_);
    for (int i=0; i<num_corners_; ++i)
    {
      std::getline(corner_file, corner_line);
      std::stringstream ss(corner_line);
      ss >> x_str >> y_str;
      xs[i] = std::stof(x_str);
      ys[i] = std::stof(y_str);
    }

    std::vector<int> indexes(num_corners_);
    for (size_t i = 0; i < indexes.size(); ++i)
    {
      indexes[i] = i;
    }
    if (num_corners_ > FLAGS_max_num_patches)
    {
      std::shuffle(indexes.begin(), indexes.end(), prng);
      num_corners_ = FLAGS_max_num_patches;
    }

    corners_P_.resize(3, num_corners_);
    bearings_P_.resize(3, num_corners_);
    corners_C_.resize(2, num_corners_);

    for (int i=0; i<num_corners_; ++i)
    {
      corners_P_.col(i) = ze::Bearing(xs[indexes[i]], ys[indexes[i]], 1.0);
    }
  }
    
  else
  {
    std::uniform_int_distribution<int> uni_x(0,239);
    std::uniform_int_distribution<int> uni_y(0,179); 
    num_corners_ = FLAGS_num_empty_patches;
    corners_P_.resize(3, num_corners_);
    bearings_P_.resize(3, num_corners_);
    corners_C_.resize(2, num_corners_);
    for (int i=0; i<num_corners_; ++i)
    {
      int x = uni_x(prng);
      int y = uni_y(prng);
      corners_P_.col(i) = ze::Bearing(x, y, 1.0);
    }
  }

  event_counters_.resize(num_corners_, 0);

  for (size_t i = 0; i < num_corners_; ++i)
  {
    std::stringstream ss;
    ss << output_folder << "_" << i << ".csv";
    std::ofstream file_out;
    file_out.open(ss.str());
    file_out << "t,x,y,p,min corner distance, min corner index" << std::endl;
    events_text_files_.push_back(std::move(file_out));
  }
}

TextFilePublisher::~TextFilePublisher()
{
  for (size_t i = 0; i < events_text_files_.size(); ++i)
  {
    events_text_files_[i].close();
  }
}

Publisher::Ptr TextFilePublisher::createFromGflags(const std::string& corner_filename, const std::string& output_folder)
{
  if(output_folder == "")
  {
    LOG(WARNING) << "Empty events text file path: will not write events to a text file.";
    return nullptr;
  }

  return std::make_shared<TextFilePublisher>(corner_filename, output_folder);
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

bool TextFilePublisher::eventsCallback(const EventsVector& events)
{
  CHECK_EQ(events.size(), 1);
  for(const Event& e : events[0])
  {
    ze::Keypoint event(e.x, e.y);
    ze::Keypoints distances = corners_C_.colwise() - event;
    int min_index;
    float min_dist = distances.colwise().norm().minCoeff(&min_index);
    for (int i=0; i<num_corners_; ++i)
    { 
      ze::Keypoint distance = distances.col(i).cwiseAbs();
      if (distance[0] < FLAGS_patch_size_half && distance[1] < FLAGS_patch_size_half)
      {
        events_text_files_[i] << e.t << "," << e.x << "," << e.y << "," << e.pol << "," << min_dist << "," << min_index << std::endl;
        event_counters_[i]++;
      }
    }
  }
  // for (int i=0; i<num_corners_; ++i)
  //   LOG(WARNING) << event_counters_[i];
  return !std::all_of(event_counters_.begin(), event_counters_.end(), [](int c) { return c > FLAGS_max_events_per_patch; });
}


} // namespace event_camera_simulator
