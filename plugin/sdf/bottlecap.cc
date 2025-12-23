// Custom bottle cap SDF plugin
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "sdf.h"
#include "bottlecap.h"

namespace mujoco::plugin::sdf {
namespace {

inline mjtNum sdCylinder(const mjtNum p[3], mjtNum r, mjtNum h) {
  mjtNum dxz = std::sqrt(p[0]*p[0] + p[1]*p[1]) - r;
  mjtNum dy = std::abs(p[2]) - h;
  mjtNum outside = std::sqrt(std::max(dxz, (mjtNum)0) * std::max(dxz, (mjtNum)0) +
                             std::max(dy, (mjtNum)0) * std::max(dy, (mjtNum)0));
  mjtNum inside = std::min(std::max(dxz, dy), (mjtNum)0);
  return outside + inside;
}

// Cap shell: outer cylinder minus inner cavity with lip offset upward
inline mjtNum triWave(mjtNum x) {
  return std::abs(Fract(x) * 2 - 1);
}

inline mjtNum helixPhase(const mjtNum p[3], mjtNum pitch) {
  mjtNum angleTurns = mju_atan2(p[1], p[0]) / (2 * mjPI);
  mjtNum axialTurns = p[2] / pitch;
  return axialTurns - angleTurns;
}

inline mjtNum threadMod(const mjtNum p[3], mjtNum pitch, mjtNum depth, mjtNum halfHeight) {
  if (std::abs(p[2]) > halfHeight) return 0;
  mjtNum phase = helixPhase(p, pitch);
  mjtNum tri = triWave(phase);
  return (1 - tri) * depth;
}

inline mjtNum capDistance(const mjtNum p[3], const mjtNum* a) {
  mjtNum R = a[0];
  mjtNum H = a[1] * 0.5;
  mjtNum lipH = a[2] * 0.5;
  mjtNum clr = a[3];
  mjtNum pitch = mju_max(a[4], (mjtNum)1e-5);
  mjtNum tDepth = a[5];

  mjtNum outer = sdCylinder(p, R, H);

  // Center the inner cavity so threads span the full usable height.
  // lipH reserves a lip at the top: the cavity height shrinks by lipH,
  // and the cavity is shifted downward by lipH to leave the top lip solid.
  mjtNum innerHalfH = mju_max((mjtNum)0, H - lipH);
  mjtNum innerP[3] = {p[0], p[1], p[2] + lipH};
  mjtNum mod = threadMod(innerP, pitch, tDepth, innerHalfH);
  mjtNum inner = sdCylinder(innerP, (R - clr) - mod, innerHalfH);

  return std::max(outer, -inner);
}

}  // namespace

std::optional<BottleCap> BottleCap::Create(const mjModel* m, mjData* d, int instance) {
  for (int i = 0; i < BottleCapAttribute::nattribute; i++) {
    if (!CheckAttr(BottleCapAttribute::names[i], m, instance)) {
      mju_warning("Invalid parameter specification in BottleCap plugin");
      return std::nullopt;
    }
  }
  return BottleCap(m, d, instance);
}

BottleCap::BottleCap(const mjModel* m, mjData* d, int instance) {
  SdfDefault<BottleCapAttribute> defattribute;
  for (int i = 0; i < BottleCapAttribute::nattribute; i++) {
    attribute[i] = defattribute.GetDefault(
      BottleCapAttribute::names[i],
      mj_getPluginConfig(m, instance, BottleCapAttribute::names[i]));
  }
}

void BottleCap::Compute(const mjModel* m, mjData* d, int instance) {
  visualizer_.Next();
}

void BottleCap::Reset() {
  visualizer_.Reset();
}

void BottleCap::Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
  visualizer_.Visualize(m, d, opt, scn, instance);
}

mjtNum BottleCap::Distance(const mjtNum point[3]) const {
  return capDistance(point, attribute);
}

void BottleCap::Gradient(mjtNum grad[3], const mjtNum point[3]) const {
  mjtNum eps = 1e-6;
  mjtNum d0 = capDistance(point, attribute);
  mjtNum p1[3] = {point[0] + eps, point[1], point[2]};
  mjtNum p2[3] = {point[0], point[1] + eps, point[2]};
  mjtNum p3[3] = {point[0], point[1], point[2] + eps};
  grad[0] = (capDistance(p1, attribute) - d0) / eps;
  grad[1] = (capDistance(p2, attribute) - d0) / eps;
  grad[2] = (capDistance(p3, attribute) - d0) / eps;
}

void BottleCap::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sdf.bottlecap";
  plugin.capabilityflags |= mjPLUGIN_SDF;
  plugin.nattribute = BottleCapAttribute::nattribute;
  plugin.attributes = BottleCapAttribute::names;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto cap = BottleCap::Create(m, d, instance);
    if (!cap) return -1;
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(new BottleCap(std::move(*cap)));
    return 0;
  };

  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<BottleCap*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data, int instance) {
    auto cap = reinterpret_cast<BottleCap*>(plugin_data);
    cap->Reset();
  };

  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
    auto cap = reinterpret_cast<BottleCap*>(d->plugin_data[instance]);
    cap->Visualize(m, d, opt, scn, instance);
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
    auto cap = reinterpret_cast<BottleCap*>(d->plugin_data[instance]);
    cap->Compute(m, d, instance);
  };

  plugin.sdf_distance = +[](const mjtNum point[3], const mjData* d, int instance) {
    auto cap = reinterpret_cast<BottleCap*>(d->plugin_data[instance]);
    return cap->Distance(point);
  };

  plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3], const mjData* d, int instance) {
    auto cap = reinterpret_cast<BottleCap*>(d->plugin_data[instance]);
    cap->visualizer_.AddPoint(point);
    cap->Gradient(gradient, point);
  };

  plugin.sdf_staticdistance = +[](const mjtNum point[3], const mjtNum* attributes) {
    return capDistance(point, attributes);
  };

  plugin.sdf_aabb = +[](mjtNum aabb[6], const mjtNum* attributes) {
    mjtNum R = attributes[0];
    mjtNum H = attributes[1] * 0.5;
    mjtNum threadDepth = attributes[5];
    // AABB defined as center (0,0,0) with half-extents on each axis.
    aabb[0] = aabb[1] = aabb[2] = 0;
    aabb[3] = R + threadDepth;
    aabb[4] = R + threadDepth;
    aabb[5] = H;
  };

  plugin.sdf_attribute = +[](mjtNum attribute[], const char* name[], const char* value[]) {
    SdfDefault<BottleCapAttribute> defattribute;
    defattribute.GetDefaults(attribute, name, value);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::sdf


