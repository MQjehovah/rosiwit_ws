#include "rosiwit_navigation/nav_core/layered_costmap.hpp"

#include <algorithm>
#include <cmath>

namespace rosiwit_navigation
{
namespace core
{

bool ICostmapLayer::worldToIndex(const LayerConfig & cfg, double wx, double wy,
                                 int & gx, int & gy)
{
  if (!cfg.valid()) { return false; }
  gx = static_cast<int>(std::floor((wx - cfg.map_origin_x) / cfg.resolution)) - cfg.window_x;
  gy = static_cast<int>(std::floor((wy - cfg.map_origin_y) / cfg.resolution)) - cfg.window_y;
  return gx >= 0 && gx < static_cast<int>(cfg.width) &&
         gy >= 0 && gy < static_cast<int>(cfg.height);
}

void LayeredCostmap::configure(const LayerConfig & cfg, int8_t default_value)
{
  cfg_ = cfg;
  default_value_ = default_value;
  resetMaster();
  for (const auto & layer : layers_) { layer->configure(cfg_); }
}

void LayeredCostmap::resetMaster()
{
  const size_t n = static_cast<size_t>(cfg_.width) * cfg_.height;
  master_.assign(n, default_value_);
}

bool LayeredCostmap::addLayer(const std::shared_ptr<ICostmapLayer> & layer)
{
  if (!layer) { return false; }
  if (hasLayer(layer->name())) { return false; }
  layer->configure(cfg_);
  layers_.push_back(layer);
  return true;
}

void LayeredCostmap::clearLayers() { layers_.clear(); }

bool LayeredCostmap::hasLayer(const std::string & name) const
{
  for (const auto & layer : layers_) {
    if (layer->name() == name) { return true; }
  }
  return false;
}

std::vector<std::string> LayeredCostmap::layerNames() const
{
  std::vector<std::string> names;
  names.reserve(layers_.size());
  for (const auto & layer : layers_) { names.push_back(layer->name()); }
  return names;
}

void LayeredCostmap::update(double now)
{
  if (!cfg_.valid()) { return; }
  if (master_.size() != static_cast<size_t>(cfg_.width) * cfg_.height) { resetMaster(); }
  resetMaster();
  for (const auto & layer : layers_) {
    layer->update(now);
    layer->stamp(cfg_, master_);
  }
}

std::vector<int8_t> LayeredCostmap::combinedOf(const std::vector<std::string> & names) const
{
  std::vector<int8_t> out(static_cast<size_t>(cfg_.width) * cfg_.height, default_value_);
  for (const auto & layer : layers_) {
    bool wanted = false;
    for (const auto & n : names) {
      if (layer->name() == n) { wanted = true; break; }
    }
    if (wanted) { layer->stamp(cfg_, out); }
  }
  return out;
}

}  // namespace core
}  // namespace rosiwit_navigation
