// Custom bottle cap SDF plugin
#pragma once

#include <optional>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {

struct BottleCapAttribute {
  static constexpr int nattribute = 6;
  static constexpr const char* names[nattribute] = {
    "radius",        // outer radius
    "height",        // total height
    "lip_height",    // inner engagement height
    "clearance",     // radial clearance
    "thread_pitch",  // thread pitch (meters per revolution)
    "thread_depth"   // radial thread depth
  };
  static constexpr mjtNum defaults[nattribute] = {
    0.03, 0.02, 0.004, 0.0005,
    0.01, 0.0015
  };
};

class BottleCap {
 public:
  static std::optional<BottleCap> Create(const mjModel* m, mjData* d, int instance);
  BottleCap(BottleCap&&) = default;
  ~BottleCap() = default;

  void Reset();
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  mjtNum Distance(const mjtNum point[3]) const;
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

  static void RegisterPlugin();

  mjtNum attribute[BottleCapAttribute::nattribute];

 private:
  BottleCap(const mjModel* m, mjData* d, int instance);
  SdfVisualizer visualizer_;
};

}  // namespace mujoco::plugin::sdf


