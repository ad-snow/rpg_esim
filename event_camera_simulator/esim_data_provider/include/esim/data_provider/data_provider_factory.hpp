#pragma once

#include <gflags/gflags.h>
#include <esim/data_provider/data_provider_base.hpp>

namespace event_camera_simulator {

DataProviderBase::Ptr loadDataProviderFromGflags(const std::string& path_to_texture);

} // namespace ze
