/**
 * @file route_water.cpp
 * @brief 使用 GEOS 检查水域覆盖、障碍碰撞及整条航道的安全净距。
 */

#include "route_water.hpp"

#include <geos_c.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace usv_map_tools
{
namespace
{
struct Context
{
  GEOSContextHandle_t handle = GEOS_init_r();
  std::string error;
  Context()
  {
    if (!handle) {
      throw std::runtime_error("cannot initialize GEOS");
    }
    GEOSContext_setErrorMessageHandler_r(handle, [](const char * message, void * user) {
      static_cast<Context *>(user)->error = message;
    }, this);
  }
  ~Context() {GEOS_finish_r(handle);}
};

struct GeometryDeleter
{
  GEOSContextHandle_t context;
  void operator()(GEOSGeometry * geometry) const {GEOSGeom_destroy_r(context, geometry);}
};
using Geometry = std::unique_ptr<GEOSGeometry, GeometryDeleter>;

class GeometryOps
{
public:
  Context context;
  GEOSContextHandle_t ctx() const {return context.handle;}
  Geometry own(GEOSGeometry * geometry) const
  {
    if (!geometry) {
      throw std::runtime_error("invalid route map geometry: " + context.error);
    }
    return Geometry(geometry, GeometryDeleter{ctx()});
  }

  Geometry line(const std::vector<RoutePoint> & points, bool ring = false) const
  {
    if (points.size() < (ring ? 3u : 2u)) {
      throw std::invalid_argument("map ring or route has too few points");
    }
    const bool close = ring && (points.front().x != points.back().x ||
      points.front().y != points.back().y);
    auto * sequence = GEOSCoordSeq_create_r(ctx(), points.size() + (close ? 1 : 0), 2);
    if (!sequence) {
      throw std::runtime_error("cannot allocate route coordinates");
    }
    for (std::size_t i = 0; i < points.size() + (close ? 1 : 0); ++i) {
      const auto p = points[i % points.size()];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !GEOSCoordSeq_setXY_r(ctx(), sequence, i, p.x, p.y))
      {
        GEOSCoordSeq_destroy_r(ctx(), sequence);
        throw std::invalid_argument("invalid route or chart coordinates");
      }
    }
    return own(ring ? GEOSGeom_createLinearRing_r(ctx(), sequence) :
      GEOSGeom_createLineString_r(ctx(), sequence));
  }

  Geometry unite(std::vector<Geometry> geometries) const
  {
    std::vector<GEOSGeometry *> raw;
    for (auto & geometry : geometries) {
      raw.push_back(geometry.release());
    }
    auto collection = own(GEOSGeom_createCollection_r(
      ctx(), GEOS_GEOMETRYCOLLECTION, raw.data(), raw.size()));
    return own(GEOSUnaryUnion_r(ctx(), collection.get()));
  }

  Geometry polygon(const usv_map::Geometry & source, std::size_t begin, std::size_t end) const
  {
    std::vector<Geometry> rings;
    for (std::size_t i = begin; i < end; ++i) {
      std::vector<RoutePoint> points;
      for (const auto & p : source.parts.at(i)) {
        points.push_back({p.x, p.y});
      }
      rings.push_back(line(points, true));
    }
    if (rings.empty()) {
      throw std::runtime_error("chart polygon has no exterior ring");
    }
    std::vector<GEOSGeometry *> holes;
    for (std::size_t i = 1; i < rings.size(); ++i) {
      holes.push_back(rings[i].release());
    }
    auto polygon = own(GEOSGeom_createPolygon_r(
      ctx(), rings.front().release(), holes.data(), holes.size()));
    // 图像轮廓可能自接触；修复后保留退化的陆地边线，水域仅保留面几何。
    return own(GEOSMakeValid_r(ctx(), polygon.get()));
  }
};
}  // namespace

struct RouteWater::Impl
{
  GeometryOps geos;
  Geometry water;
  Geometry land;
  Geometry boundary;
  Geometry safe;
  const GEOSPreparedGeometry * prepared = nullptr;

  Impl(const usv_map::Chart & chart, double margin)
  : water(nullptr, GeometryDeleter{geos.ctx()}),
    land(nullptr, GeometryDeleter{geos.ctx()}),
    boundary(nullptr, GeometryDeleter{geos.ctx()}),
    safe(nullptr, GeometryDeleter{geos.ctx()})
  {
    if (!std::isfinite(margin) || margin < 0.0) {
      throw std::invalid_argument("clearance_m must be finite and nonnegative");
    }
    if (chart.has_unreadable_navigation_geometry) {
      throw std::runtime_error("chart contains unreadable water/land geometry; refusing unsafe route");
    }
    std::vector<Geometry> water_parts, land_parts;
    for (const auto & feature : chart.features) {
      std::string group = feature.group;
      std::transform(group.begin(), group.end(), group.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
      });
      if (group != "DEPARE" && group != "LNDARE") {
        continue;
      }
      if (feature.geometry.empty()) {
        throw std::runtime_error("water/land feature contains no readable geometry");
      }
      for (const auto & geometry : feature.geometry) {
        if (geometry.type != "POLYGON" && geometry.type != "MULTIPOLYGON") {
          throw std::runtime_error("water/land feature must contain polygon geometry");
        }
        auto starts = geometry.polygon_starts;
        if (starts.empty() && geometry.type == "POLYGON") {
          starts.push_back(0);
        }
        if (starts.empty() || geometry.parts.empty()) {
          throw std::runtime_error("water/land polygon is empty or has no ring grouping");
        }
        starts.push_back(geometry.parts.size());
        for (std::size_t i = 1; i < starts.size(); ++i) {
          auto part = geos.polygon(geometry, starts[i - 1], starts[i]);
          if (group == "DEPARE") {
            water_parts.push_back(geos.own(GEOSBuffer_r(geos.ctx(), part.get(), 0.0, 8)));
          } else {
            land_parts.push_back(std::move(part));
          }
        }
      }
    }
    auto all_water = geos.unite(std::move(water_parts));
    land = geos.unite(std::move(land_parts));
    water = geos.own(GEOSDifference_r(geos.ctx(), all_water.get(), land.get()));
    if (GEOSisEmpty_r(geos.ctx(), water.get()) != 0) {
      throw std::runtime_error("chart contains no navigable DEPARE water");
    }
    boundary = geos.own(GEOSBoundary_r(geos.ctx(), water.get()));
    auto inner = geos.own(GEOSBuffer_r(geos.ctx(), water.get(), -margin, 16));
    auto obstacles = geos.own(GEOSBuffer_r(geos.ctx(), land.get(), margin, 16));
    safe = geos.own(GEOSDifference_r(geos.ctx(), inner.get(), obstacles.get()));
    if (GEOSisEmpty_r(geos.ctx(), safe.get()) != 0) {
      throw std::runtime_error("no water remains at the requested clearance");
    }
    prepared = GEOSPrepare_r(geos.ctx(), safe.get());
    if (!prepared) {
      throw std::runtime_error("cannot prepare route water geometry");
    }
  }

  ~Impl()
  {
    if (prepared) {
      GEOSPreparedGeom_destroy_r(geos.ctx(), prepared);
    }
  }
};

RouteWater::RouteWater(const usv_map::Chart & chart, double clearance_m)
: impl_(std::make_unique<Impl>(chart, clearance_m)) {}
RouteWater::~RouteWater() = default;

double RouteWater::unsafe_length(const std::vector<RoutePoint> & points) const
{
  const auto & map = *impl_;
  auto line = map.geos.line(points);
  const auto covered = GEOSPreparedCovers_r(map.geos.ctx(), map.prepared, line.get());
  if (covered == 1) {
    return 0.0;
  }
  if (covered != 0) {
    throw std::runtime_error("route water coverage check failed");
  }
  auto outside = map.geos.own(GEOSDifference_r(map.geos.ctx(), line.get(), map.safe.get()));
  double length = 0.0;
  if (!GEOSLength_r(map.geos.ctx(), outside.get(), &length) || !std::isfinite(length)) {
    throw std::runtime_error("route unsafe length check failed");
  }
  // 水域外的相切点长度为零，仍属于越界，必须拒绝。
  return std::max(1e-6, length);
}

double RouteWater::clearance(const std::vector<RoutePoint> & points) const
{
  const auto & map = *impl_;
  const auto ctx = map.geos.ctx();
  auto line = map.geos.line(points);
  if (GEOSCovers_r(ctx, map.water.get(), line.get()) != 1 ||
    GEOSDisjoint_r(ctx, map.land.get(), line.get()) != 1)
  {
    return -1.0;
  }
  double distance = 0.0;
  if (!GEOSDistance_r(ctx, map.boundary.get(), line.get(), &distance)) {
    throw std::runtime_error("route shoreline distance check failed");
  }
  if (GEOSisEmpty_r(ctx, map.land.get()) == 0) {
    double land_distance = 0.0;
    if (!GEOSDistance_r(ctx, map.land.get(), line.get(), &land_distance)) {
      throw std::runtime_error("route obstacle distance check failed");
    }
    distance = std::min(distance, land_distance);
  }
  if (!std::isfinite(distance)) {
    throw std::runtime_error("non-finite route clearance");
  }
  return distance;
}

double RouteWater::corridor_clearance(
  const std::vector<RoutePoint> & outer, const std::vector<RoutePoint> & inner) const
{
  const auto & map = *impl_;
  const auto ctx = map.geos.ctx();
  auto shell = map.geos.line(outer, true), hole = map.geos.line(inner, true);
  auto * hole_raw = hole.release();
  auto polygon = map.geos.own(GEOSGeom_createPolygon_r(ctx, shell.release(), &hole_raw, 1));
  if (GEOSisValid_r(ctx, polygon.get()) != 1) {
    throw std::runtime_error("fairway boundaries cross or form an invalid corridor");
  }
  if (GEOSCovers_r(ctx, map.water.get(), polygon.get()) != 1 ||
    GEOSDisjoint_r(ctx, map.land.get(), polygon.get()) != 1)
  {
    return -1.0;
  }
  double distance = 0.0;
  if (!GEOSDistance_r(ctx, map.boundary.get(), polygon.get(), &distance)) {
    throw std::runtime_error("fairway shoreline distance check failed");
  }
  if (GEOSisEmpty_r(ctx, map.land.get()) == 0) {
    double land_distance = 0;
    if (!GEOSDistance_r(ctx, map.land.get(), polygon.get(), &land_distance)) {
      throw std::runtime_error("fairway obstacle distance check failed");
    }
    distance = std::min(distance, land_distance);
  }
  if (!std::isfinite(distance)) {
    throw std::runtime_error("non-finite fairway clearance");
  }
  return distance;
}
}  // namespace usv_map_tools
