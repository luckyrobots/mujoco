// Custom bottle SDF plugin (hollow interior for cap fit)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>

#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "sdf.h"
#include "bottle.h"

namespace mujoco::plugin::sdf {
namespace {

inline mjtNum radialFacetScale(mjtNum x, mjtNum y, int sides) {
  sides = std::max(3, sides);
  mjtNum k = mjPI / sides;
  mjtNum a = mju_atan2(y, x);
  mjtNum sector = std::fmod(a + mjPI, 2 * k) - k;
  return std::cos(k) / std::cos(sector);
}

inline mjtNum sdCylinder(const mjtNum p[3], mjtNum r, mjtNum h) {
  mjtNum dxz = std::sqrt(p[0]*p[0] + p[1]*p[1]) - r;
  mjtNum dy = std::abs(p[2]) - h;
  mjtNum outside = std::sqrt(std::max(dxz, (mjtNum)0) * std::max(dxz, (mjtNum)0) +
                             std::max(dy, (mjtNum)0) * std::max(dy, (mjtNum)0));
  mjtNum inside = std::min(std::max(dxz, dy), (mjtNum)0);
  return outside + inside;
}

inline mjtNum sdNgonCylinder(const mjtNum p[3], mjtNum r, mjtNum h, int sides) {
  mjtNum factor = radialFacetScale(p[0], p[1], sides);
  mjtNum radial = std::sqrt(p[0]*p[0] + p[1]*p[1]);
  mjtNum dxz = radial - r * factor;
  mjtNum dy = std::abs(p[2]) - h;
  mjtNum outside = std::sqrt(std::max(dxz, (mjtNum)0) * std::max(dxz, (mjtNum)0) +
                             std::max(dy, (mjtNum)0) * std::max(dy, (mjtNum)0));
  mjtNum inside = std::min(std::max(dxz, dy), (mjtNum)0);
  return outside + inside;
}

// Bottle interior as union of body cylinder and neck cylinder (negative inside)
inline mjtNum triWave(mjtNum x) {
  // triangular wave in [0,1]
  return std::abs(Fract(x) * 2 - 1);
}

inline mjtNum helixPhase(const mjtNum p[3], mjtNum pitch) {
  // couple axial travel with azimuth to form a continuous helix
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

inline mjtNum bottleDistance(const mjtNum p[3], const mjtNum* a) {
  mjtNum bodyR = a[0] - a[4];
  mjtNum bodyH = a[1] * 0.5;
  mjtNum neckR = a[2] - a[4];
  mjtNum neckH = a[3] * 0.5;
  mjtNum pitch = mju_max(a[5], (mjtNum)1e-5);
  mjtNum tDepth = a[6];
  int sides = std::max(3, static_cast<int>(std::lround(a[7])));

  // body centered at z=0, neck sits on top (shifted up by bodyH + neckH)
  mjtNum pb[3] = {p[0], p[1], p[2]};
  mjtNum pn[3] = {p[0], p[1], p[2] - (bodyH + neckH)};

  mjtNum dBody = sdNgonCylinder(pb, bodyR, bodyH, sides);

  // External thread on neck: modulate radius outward. Neck stays circular.
  mjtNum mod = threadMod(pn, pitch, tDepth, neckH);
  mjtNum dNeck = sdCylinder(pn, neckR + mod, neckH);

  return std::min(dBody, dNeck);
}

}  // namespace

std::optional<Bottle> Bottle::Create(const mjModel* m, mjData* d, int instance) {
  for (int i = 0; i < BottleAttribute::nattribute; i++) {
    if (!CheckAttr(BottleAttribute::names[i], m, instance)) {
      mju_warning("Invalid parameter specification in Bottle plugin");
      return std::nullopt;
    }
  }
  return Bottle(m, d, instance);
}

Bottle::Bottle(const mjModel* m, mjData* d, int instance) {
  SdfDefault<BottleAttribute> defattribute;
  for (int i = 0; i < BottleAttribute::nattribute; i++) {
    attribute[i] = defattribute.GetDefault(
      BottleAttribute::names[i],
      mj_getPluginConfig(m, instance, BottleAttribute::names[i]));
  }
}

void Bottle::Compute(const mjModel* m, mjData* d, int instance) {
  visualizer_.Next();
}

void Bottle::Reset() {
  visualizer_.Reset();
}

void Bottle::Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
  visualizer_.Visualize(m, d, opt, scn, instance);
}

mjtNum Bottle::Distance(const mjtNum point[3]) const {
  return bottleDistance(point, attribute);
}

void Bottle::Gradient(mjtNum grad[3], const mjtNum point[3]) const {
  mjtNum eps = 1e-6;
  mjtNum d0 = bottleDistance(point, attribute);
  mjtNum p1[3] = {point[0] + eps, point[1], point[2]};
  mjtNum p2[3] = {point[0], point[1] + eps, point[2]};
  mjtNum p3[3] = {point[0], point[1], point[2] + eps};
  grad[0] = (bottleDistance(p1, attribute) - d0) / eps;
  grad[1] = (bottleDistance(p2, attribute) - d0) / eps;
  grad[2] = (bottleDistance(p3, attribute) - d0) / eps;
}

void Bottle::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.sdf.bottle";
  plugin.capabilityflags |= mjPLUGIN_SDF;
  plugin.nattribute = BottleAttribute::nattribute;
  plugin.attributes = BottleAttribute::names;
  plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto bottle = Bottle::Create(m, d, instance);
    if (!bottle) return -1;
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(new Bottle(std::move(*bottle)));
    return 0;
  };

  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Bottle*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data, int instance) {
    auto bottle = reinterpret_cast<Bottle*>(plugin_data);
    bottle->Reset();
  };

  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
    auto bottle = reinterpret_cast<Bottle*>(d->plugin_data[instance]);
    bottle->Visualize(m, d, opt, scn, instance);
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
    auto bottle = reinterpret_cast<Bottle*>(d->plugin_data[instance]);
    bottle->Compute(m, d, instance);
  };

  plugin.sdf_distance = +[](const mjtNum point[3], const mjData* d, int instance) {
    auto bottle = reinterpret_cast<Bottle*>(d->plugin_data[instance]);
    return bottle->Distance(point);
  };

  plugin.sdf_gradient = +[](mjtNum gradient[3], const mjtNum point[3], const mjData* d, int instance) {
    auto bottle = reinterpret_cast<Bottle*>(d->plugin_data[instance]);
    bottle->visualizer_.AddPoint(point);
    bottle->Gradient(gradient, point);
  };

  plugin.sdf_staticdistance = +[](const mjtNum point[3], const mjtNum* attributes) {
    return bottleDistance(point, attributes);
  };

  plugin.sdf_aabb = +[](mjtNum aabb[6], const mjtNum* attributes) {
    mjtNum bodyR = attributes[0];
    mjtNum bodyH = attributes[1] * 0.5;
    mjtNum neckR = attributes[2];
    mjtNum neckH = attributes[3] * 0.5;
    mjtNum R = std::max(bodyR, neckR);
    mjtNum H = bodyH + neckH;
    mjtNum threadDepth = attributes[6];
    // AABB defined as center (0,0,0) with half-extents on each axis.
    aabb[0] = aabb[1] = aabb[2] = 0;
    aabb[3] = R + threadDepth;
    aabb[4] = R + threadDepth;
    aabb[5] = H + neckH; // include neck extension above body
  };

  plugin.sdf_attribute = +[](mjtNum attribute[], const char* name[], const char* value[]) {
    SdfDefault<BottleAttribute> defattribute;
    defattribute.GetDefaults(attribute, name, value);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::sdf


