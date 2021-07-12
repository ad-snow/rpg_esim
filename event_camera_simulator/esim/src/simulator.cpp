#include <esim/esim/simulator.hpp>
#include <ze/common/timer_collection.hpp>
#include <esim/common/utils.hpp>

namespace event_camera_simulator {

DECLARE_TIMER(TimerEventSimulator, timers_event_simulator_,
              simulate_events,
              visualization
              );

Simulator::~Simulator()
{
  timers_event_simulator_.saveToFile("/tmp", "event_simulator.csv");
}

void Simulator::setMaxEvents(const int max) 
{
  max_events_ = max;
}

bool Simulator::dataProviderCallback(const SimulatorData &sim_data)
{
  CHECK_EQ(event_simulators_.size(), num_cameras_);
  
  // LOG(WARNING) << "Simulator: " << sim_data.renderer_info_updated;
  bool camera_simulator_success;
  bool should_continue = true;

  if(sim_data.images_updated)
  {
    EventsVector events(num_cameras_);
    Time time = sim_data.timestamp;
    // simulate the events and camera images for every sensor in the rig
    {
      auto t = timers_event_simulator_[TimerEventSimulator::simulate_events].timeScope();
      for(size_t i=0; i<num_cameras_; ++i)
      {
        events[i] = event_simulators_[i].imageCallback(*sim_data.images[i], time);

        if(corrupted_camera_images_.size() < num_cameras_)
        {
          // allocate memory for the corrupted camera images and set them to 0
          corrupted_camera_images_.emplace_back(std::make_shared<Image>(sim_data.images[i]->size()));
          corrupted_camera_images_[i]->setTo(0.);
        }

        camera_simulator_success = camera_simulators_[i].imageCallback(*sim_data.images[i], time, corrupted_camera_images_[i]);
      }
    }

    // publish the simulation data + events
    {
      auto t = timers_event_simulator_[TimerEventSimulator::visualization].timeScope();
      publishData(sim_data, events, camera_simulator_success, corrupted_camera_images_);
    }
    events_counter_ += events[0].size();
    if (max_events_ > 0 && events_counter_ > max_events_)
    {
      should_continue = false;
    }
    else
    {
      // LOG(WARNING) << max_events_ << "," << events_counter_;
    }
  }
  else
  {
    {
      // just forward the simulation data to the publisher
      auto t = timers_event_simulator_[TimerEventSimulator::visualization].timeScope();
      publishData(sim_data, {}, camera_simulator_success, corrupted_camera_images_);
    }
  }
  return should_continue;
}

void Simulator::publishData(const SimulatorData& sim_data,
                            const EventsVector& events,
                            bool camera_simulator_success,
                            const ImagePtrVector& camera_images)
{
  if(publishers_.empty())
  {
    LOG_FIRST_N(WARNING, 1) << "No publisher available";
    return;
  }

  Time time = sim_data.timestamp;
  const Transformation& T_W_B = sim_data.groundtruth.T_W_B;
  const TransformationVector& T_W_Cs = sim_data.groundtruth.T_W_Cs;
  const ze::CameraRig::Ptr& camera_rig = sim_data.camera_rig;

  // Publish the new data (events, images, depth maps, poses, point clouds, etc.)
  // LOG(WARNING) << "camera info";
  if(camera_rig)
  {
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->cameraInfoCallback(camera_rig, time);
  }
  // LOG(WARNING) << "renderer info";
  if(sim_data.renderer_info_updated)
  {
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->rendererInfoCallback(sim_data.renderer.T_W_P_, sim_data.renderer.K_inv_, time);
  }
  // else{
  //   LOG(WARNING) << "renderer info not updated";
  // }
  // LOG(WARNING) << "pose";
  if(sim_data.poses_updated)
  {
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->poseCallback(T_W_B, T_W_Cs, time);
  }
  // LOG(WARNING) << "events";
  if(!events.empty())
  {
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->eventsCallback(events);
  }
  if(sim_data.twists_updated)
  {
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->twistCallback(sim_data.groundtruth.angular_velocities_,
                              sim_data.groundtruth.linear_velocities_,
                              time);
  }
  if(sim_data.imu_updated)
  {
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->imuCallback(sim_data.specific_force_corrupted, sim_data.angular_velocity_corrupted, time);
  }
  if(sim_data.images_updated)
  {
    for(const Publisher::Ptr& publisher : publishers_)
    {
      publisher->imageCallback(sim_data.images, time);

      if(camera_simulator_success && time >= exposure_time_)
      {
        // the images should be timestamped at mid-exposure
        const Time mid_exposure_time = time - 0.5 * exposure_time_;
        publisher->imageCorruptedCallback(camera_images, mid_exposure_time);
      }
    }
  }
  if(sim_data.depthmaps_updated)
  {
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->depthmapCallback(sim_data.depthmaps, time);
  }
  if(sim_data.optic_flows_updated)
  {
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->opticFlowCallback(sim_data.optic_flows, time);
  }
  if(sim_data.depthmaps_updated && !events.empty())
  {
    PointCloudVector pointclouds(num_cameras_);
    for(size_t i=0; i<num_cameras_; ++i)
    {
      CHECK(sim_data.depthmaps[i]);
      pointclouds[i] = eventsToPointCloud(events[i], *sim_data.depthmaps[i], camera_rig->atShared(i));
    }
    for(const Publisher::Ptr& publisher : publishers_)
      publisher->pointcloudCallback(pointclouds, time);
  }
}

} // namespace event_camera_simulator
