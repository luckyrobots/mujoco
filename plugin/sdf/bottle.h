// Custom bottle SDF plugin (hollow bottle interior)
#pragma once

#include <optional>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include "sdf.h"

namespace mujoco::plugin::sdf {

struct BottleAttribute {
  static constexpr int nattribute = 7;
  static constexpr const char* names[nattribute] = {
    "body_radius",   // outer body radius
    "body_height",   // body height
    "neck_radius",   // neck outer radius
    "neck_height",   // neck height
    "clearance",     // inner clearance for cap fit
    "thread_pitch",  // thread pitch (meters per revolution)
    "thread_depth"   // radial thread depth
  };
  static constexpr mjtNum defaults[nattribute] = {
    0.04, 0.20, 0.03, 0.03, 0.0005,
    0.01, 0.0015
  };
};

class Bottle {
 public:
  static std::optional<Bottle> Create(const mjModel* m, mjData* d, int instance);
  Bottle(Bottle&&) = default;
  ~Bottle() = default;

  void Reset();
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  mjtNum Distance(const mjtNum point[3]) const;
  void Gradient(mjtNum grad[3], const mjtNum point[3]) const;

  static void RegisterPlugin();

  mjtNum attribute[BottleAttribute::nattribute];

 private:
  Bottle(const mjModel* m, mjData* d, int instance);
  SdfVisualizer visualizer_;
};

}  // namespace mujoco::plugin::sdf


