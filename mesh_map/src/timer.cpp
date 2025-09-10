#include <mesh_map/timer.h>
#include <mutex>
#include <fstream>

namespace mesh_map
{

void LayerTimer::enable()
{
  std::lock_guard guard(instance().mutex_);
  instance().enabled_ = true;
  instance().file_.open("layer_timings.csv");
}

void LayerTimer::disable()
{
  std::lock_guard guard(instance().mutex_);
  instance().enabled_ = false;
  instance().file_.close();
}

void LayerTimer::recordUpdateDuration(
  const std::string& layer,
  const rclcpp::Time& timestamp,
  const Duration& locking,
  const Duration& update,
  const Duration& notify
)
{
  std::lock_guard guard(instance().mutex_);
  if (!instance().enabled_)
  {
    return;
  }

  if (!instance().file_.is_open())
  {
    instance().file_.open("layer_timings.csv");
  }

  std::ofstream& file = instance().file_;
  constexpr const char sep = ';';
  file << timestamp.nanoseconds() << sep;
  file << layer << sep;
  file << locking.count() << sep;
  file << update.count() << sep;
  file << notify.count() << '\n';
  file << std::flush;
}

} // mesh_map
