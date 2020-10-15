/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "modules/map/hdmap/adapter/proto_organizer.h"

#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"

namespace {

std::string CreateOverlapId() {
  static int count = 0;
  ++count;
  return "overlap_" + std::to_string(count);
}

}  // namespace

namespace apollo {
namespace hdmap {
namespace adapter {

using apollo::common::util::PairHash;

void ProtoOrganizer::GetRoadElements(std::vector<RoadInternal>* roads) {
  for (auto& road_internal : *roads) {
    // lanes
    for (auto& section_internal : road_internal.sections) {
      for (auto& lane_internal : section_internal.lanes) {
        std::string lane_id = lane_internal.lane.id().id();
        proto_data_.pb_lanes[lane_id] = lane_internal.lane;
        section_internal.section.add_lane_id()->set_id(lane_id);
      }
      (*road_internal.road.add_section()) = section_internal.section;
      proto_data_.pb_roads[road_internal.id] = road_internal.road;
    }
    // crosswalks
    for (auto& crosswalk : road_internal.crosswalks) {
      proto_data_.pb_crosswalks[crosswalk.id().id()] = crosswalk;
    }
    // parking_spaces
    for (auto& parking_space : road_internal.parking_spaces) {
      proto_data_.pb_parking_spaces[parking_space.id().id()] = parking_space;
    }
    // clear areas
    for (auto& clear_area : road_internal.clear_areas) {
      proto_data_.pb_clear_areas[clear_area.id().id()] = clear_area;
    }
    // speed_bump
    for (auto& speed_bump : road_internal.speed_bumps) {
      proto_data_.pb_speed_bumps[speed_bump.id().id()] = speed_bump;
    }
    // stop lines
    for (auto& stop_line_internal : road_internal.stop_lines) {
      proto_data_.pb_stop_lines[stop_line_internal.id] = stop_line_internal;
    }
    // traffic_lights
    for (auto& traffic_light_internal : road_internal.traffic_lights) {
      auto& traffic_light = traffic_light_internal.traffic_light;
      for (auto stop_line_id : traffic_light_internal.stop_line_ids) {
        CHECK_GT(proto_data_.pb_stop_lines.count(stop_line_id), 0);
        auto& stop_line_curve = proto_data_.pb_stop_lines[stop_line_id].curve;
        (*traffic_light.add_stop_line()) = stop_line_curve;
      }
      proto_data_.pb_signals[traffic_light.id().id()] = traffic_light;
    }
    // stop signs
    for (auto& stop_sign_internal : road_internal.stop_signs) {
      auto& stop_sign = stop_sign_internal.stop_sign;
      for (auto stop_line_id : stop_sign_internal.stop_line_ids) {
        CHECK_GT(proto_data_.pb_stop_lines.count(stop_line_id), 0);
        auto& stop_line_curve = proto_data_.pb_stop_lines[stop_line_id].curve;
        (*stop_sign.add_stop_line()) = stop_line_curve;
      }
      proto_data_.pb_stop_signs[stop_sign.id().id()] = stop_sign;
    }
    // yield signs
    for (auto& yield_sign_internal : road_internal.yield_signs) {
      auto& yield_sign = yield_sign_internal.yield_sign;
      for (auto stop_line_id : yield_sign_internal.stop_line_ids) {
        CHECK_GT(proto_data_.pb_stop_lines.count(stop_line_id), 0);
        auto& stop_line_curve = proto_data_.pb_stop_lines[stop_line_id].curve;
        (*yield_sign.add_stop_line()) = stop_line_curve;
      }
      proto_data_.pb_yield_signs[yield_sign.id().id()] = yield_sign;
    }
  }
}

void ProtoOrganizer::GetJunctionElements(
    const std::vector<JunctionInternal>& junctions) {
  for (auto& junction_internal : junctions) {
    std::string junction_id = junction_internal.junction.id().id();
    proto_data_.pb_junctions[junction_id] = junction_internal.junction;
  }
}

void ProtoOrganizer::GetLaneObjectOverlapElements(
    const std::string& lane_id,
    const std::vector<OverlapWithLane>& overlap_with_lanes) {
  for (auto& overlap_object : overlap_with_lanes) {
    std::string object_id = overlap_object.object_id;
    if (proto_data_.pb_crosswalks.count(object_id) <= 0 &&
        proto_data_.pb_clear_areas.count(object_id) <= 0 &&
        proto_data_.pb_speed_bumps.count(object_id) <= 0 &&
        proto_data_.pb_parking_spaces.count(object_id) <= 0) {
      continue;
    }
    PbOverlap overlap;
    std::string overlap_id = CreateOverlapId();
    proto_data_.pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
    overlap.mutable_id()->set_id(overlap_id);
    PbObjectOverlapInfo* object_overlap = overlap.add_object();
    object_overlap->mutable_id()->set_id(lane_id);
    object_overlap->mutable_lane_overlap_info()->set_start_s(
        overlap_object.start_s);
    object_overlap->mutable_lane_overlap_info()->set_end_s(
        overlap_object.end_s);
    object_overlap->mutable_lane_overlap_info()->set_is_merge(
        overlap_object.is_merge);
    object_overlap = overlap.add_object();
    object_overlap->mutable_id()->set_id(object_id);
    if (proto_data_.pb_crosswalks.count(object_id) > 0) {
      proto_data_.pb_crosswalks[object_id].add_overlap_id()->set_id(overlap_id);
      object_overlap->mutable_crosswalk_overlap_info();
    } else if (proto_data_.pb_clear_areas.count(object_id) > 0) {
      object_overlap->mutable_clear_area_overlap_info();
      proto_data_.pb_clear_areas[object_id].add_overlap_id()->set_id(
          overlap_id);
    } else if (proto_data_.pb_speed_bumps.count(object_id)) {
      object_overlap->mutable_speed_bump_overlap_info();
      proto_data_.pb_speed_bumps[object_id].add_overlap_id()->set_id(
          overlap_id);
    } else if (proto_data_.pb_parking_spaces.count(object_id)) {
      object_overlap->mutable_parking_space_overlap_info();
      proto_data_.pb_parking_spaces[object_id].add_overlap_id()->set_id(
          overlap_id);
    } else {
      AERROR << "unknown object, object id:" << object_id;
    }
    proto_data_.pb_overlaps[overlap_id] = overlap;
  }
}

void ProtoOrganizer::GetLaneSignalOverlapElements(
    const std::string& lane_id,
    const std::vector<OverlapWithLane>& overlap_with_lanes) {
  for (auto& overlap_signal : overlap_with_lanes) {
    std::string object_id = overlap_signal.object_id;
    if (proto_data_.pb_signals.count(object_id) <= 0 &&
        proto_data_.pb_stop_signs.count(object_id) <= 0 &&
        proto_data_.pb_yield_signs.count(object_id) <= 0) {
      AINFO << "cannot find signal object_id:" << object_id;
      continue;
    }
    PbOverlap overlap;
    std::string overlap_id = CreateOverlapId();
    proto_data_.pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
    overlap.mutable_id()->set_id(overlap_id);
    PbObjectOverlapInfo* object_overlap = overlap.add_object();
    object_overlap->mutable_id()->set_id(lane_id);
    object_overlap->mutable_lane_overlap_info()->set_start_s(
        overlap_signal.start_s);
    object_overlap->mutable_lane_overlap_info()->set_end_s(
        overlap_signal.end_s);
    object_overlap->mutable_lane_overlap_info()->set_is_merge(
        overlap_signal.is_merge);
    object_overlap = overlap.add_object();
    object_overlap->mutable_id()->set_id(object_id);
    if (proto_data_.pb_signals.count(object_id) > 0) {
      object_overlap->mutable_signal_overlap_info();
      proto_data_.pb_signals[object_id].add_overlap_id()->set_id(overlap_id);
    } else if (proto_data_.pb_stop_signs.count(object_id) > 0) {
      object_overlap->mutable_stop_sign_overlap_info();
      proto_data_.pb_stop_signs[object_id].add_overlap_id()->set_id(overlap_id);
    } else if (proto_data_.pb_yield_signs.count(object_id) > 0) {
      object_overlap->mutable_yield_sign_overlap_info();
      proto_data_.pb_yield_signs[object_id].add_overlap_id()->set_id(
          overlap_id);
    } else {
      AERROR << "unknown signal, signal id:" << object_id;
    }
    proto_data_.pb_overlaps[overlap_id] = overlap;
  }
}

void ProtoOrganizer::GetLaneJunctionOverlapElements(
    const std::string& lane_id,
    const std::vector<OverlapWithLane>& overlap_with_lanes) {
  for (auto& overlap_junction : overlap_with_lanes) {
    std::string object_id = overlap_junction.object_id;
    if (proto_data_.pb_junctions.count(object_id) <= 0) {
      AINFO << "cannot find junction object " << object_id;
      continue;
    }
    PbOverlap overlap;
    std::string overlap_id = CreateOverlapId();
    proto_data_.pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
    overlap.mutable_id()->set_id(overlap_id);
    PbObjectOverlapInfo* object_overlap = overlap.add_object();
    object_overlap->mutable_id()->set_id(lane_id);
    object_overlap->mutable_lane_overlap_info()->set_start_s(
        overlap_junction.start_s);
    object_overlap->mutable_lane_overlap_info()->set_end_s(
        overlap_junction.end_s);
    object_overlap->mutable_lane_overlap_info()->set_is_merge(
        overlap_junction.is_merge);
    object_overlap = overlap.add_object();
    object_overlap->mutable_id()->set_id(object_id);
    if (proto_data_.pb_junctions.count(object_id) > 0) {
      object_overlap->mutable_junction_overlap_info();
      proto_data_.pb_junctions[object_id].add_overlap_id()->set_id(overlap_id);
    } else {
      AERROR << "unknown junction overlap, id:" << object_id;
    }
    proto_data_.pb_overlaps[overlap_id] = overlap;
  }
}

void ProtoOrganizer::GetLaneLaneOverlapElements(
    const std::unordered_map<std::pair<std::string, std::string>,
                             OverlapWithLane, PairHash>& lane_lane_overlaps) {
  std::unordered_set<std::string> close_set;
  for (auto& overlap_lane_pair : lane_lane_overlaps) {
    auto& lane_id = overlap_lane_pair.first.first;
    auto& overlap_lane = overlap_lane_pair.second;
    std::string object_id = overlap_lane.object_id;
    std::string unique_object_id = lane_id + "_" + object_id;
    if (close_set.count(unique_object_id) > 0) {
      continue;
    }
    unique_object_id = object_id + "_" + lane_id;
    if (close_set.count(unique_object_id) > 0) {
      continue;
    }
    close_set.insert(unique_object_id);
    PbOverlap overlap;
    std::string overlap_id = CreateOverlapId();
    proto_data_.pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
    overlap.mutable_id()->set_id(overlap_id);
    PbObjectOverlapInfo* object_overlap = overlap.add_object();
    object_overlap->mutable_id()->set_id(lane_id);
    object_overlap->mutable_lane_overlap_info()->set_start_s(
        overlap_lane.start_s);
    object_overlap->mutable_lane_overlap_info()->set_end_s(overlap_lane.end_s);
    object_overlap->mutable_lane_overlap_info()->set_is_merge(
        overlap_lane.is_merge);
    object_overlap = overlap.add_object();
    object_overlap->mutable_id()->set_id(object_id);
    if (proto_data_.pb_lanes.count(object_id) <= 0) {
      AERROR << "unknown overlap lane, id:" << object_id;
      continue;
    }

    if (lane_lane_overlaps.count(make_pair(object_id, lane_id)) <= 0) {
      AERROR << "lane overlap is not symmetrical " << overlap_id;
      continue;
    }
    auto& lane_lane_overlap =
        lane_lane_overlaps.at(make_pair(object_id, lane_id));
    object_overlap->mutable_lane_overlap_info()->set_start_s(
        lane_lane_overlap.start_s);
    object_overlap->mutable_lane_overlap_info()->set_end_s(
        lane_lane_overlap.end_s);
    object_overlap->mutable_lane_overlap_info()->set_is_merge(
        lane_lane_overlap.is_merge);
    proto_data_.pb_lanes[object_id].add_overlap_id()->set_id(overlap_id);
    proto_data_.pb_overlaps[overlap_id] = overlap;
  }
}

void ProtoOrganizer::GetJunctionObjectOverlapElements(
    const std::vector<JunctionInternal>& junctions) {
  for (auto& junction_internal : junctions) {
    const auto& junction_id = junction_internal.junction.id().id();
    for (auto& overlap_junction : junction_internal.overlap_with_junctions) {
      PbOverlap overlap;
      std::string overlap_id = CreateOverlapId();
      proto_data_.pb_junctions[junction_id].add_overlap_id()->set_id(
          overlap_id);
      overlap.mutable_id()->set_id(overlap_id);
      PbObjectOverlapInfo* object_overlap = overlap.add_object();
      object_overlap->mutable_id()->set_id(junction_id);
      object_overlap->mutable_junction_overlap_info();
      std::string object_id = overlap_junction.object_id;
      object_overlap = overlap.add_object();
      object_overlap->mutable_id()->set_id(object_id);
      if (proto_data_.pb_crosswalks.count(object_id) > 0) {
        object_overlap->mutable_crosswalk_overlap_info();
        proto_data_.pb_crosswalks[object_id].add_overlap_id()->set_id(
            overlap_id);
      } else if (proto_data_.pb_clear_areas.count(object_id) > 0) {
        object_overlap->mutable_clear_area_overlap_info();
        proto_data_.pb_clear_areas[object_id].add_overlap_id()->set_id(
            overlap_id);
      } else if (proto_data_.pb_stop_signs.count(object_id) > 0) {
        object_overlap->mutable_stop_sign_overlap_info();
        proto_data_.pb_stop_signs[object_id].add_overlap_id()->set_id(
            overlap_id);
      } else {
        continue;
      }
      proto_data_.pb_overlaps[overlap_id] = overlap;
    }
  }
}

void ProtoOrganizer::GetOverlapElements(
    const std::vector<RoadInternal>& roads,
    const std::vector<JunctionInternal>& junctions) {
  std::unordered_map<std::pair<std::string, std::string>, OverlapWithLane,
                     PairHash>
      lane_lane_overlaps;
  // overlap
  for (auto& road_internal : roads) {
    for (auto& road_section : road_internal.sections) {
      for (auto& lane_internal : road_section.lanes) {
        std::string lane_id = lane_internal.lane.id().id();
        GetLaneObjectOverlapElements(lane_id, lane_internal.overlap_objects);
        GetLaneSignalOverlapElements(lane_id, lane_internal.overlap_signals);
        GetLaneJunctionOverlapElements(lane_id,
                                       lane_internal.overlap_junctions);
        for (auto& overlap_lane : lane_internal.overlap_lanes) {
          lane_lane_overlaps[make_pair(lane_id, overlap_lane.object_id)] =
              overlap_lane;
        }
      }
    }
  }

  GetLaneLaneOverlapElements(lane_lane_overlaps);
  GetJunctionObjectOverlapElements(junctions);
}

void ProtoOrganizer::OutputDataAllride(allride::hdmap::BaseMap& pb_map) {
  // for (auto& road_pair : proto_data_.pb_roads) {
  //   *(pb_map->add_road()) = road_pair.second;
  // }

  static int64_t lane_cnt = 20000;
  static int64_t boundary_cnt = 30000;
  static int64_t junction_cnt = 40000;

  for (auto& lane_pair : proto_data_.pb_lanes) {
    PbLane pb_lane = lane_pair.second;
    APbLane lane;
    APbBoundary boundary_left;
    APbBoundary boundary_right;

    if (pb_lane.type() != PbLaneType::Lane_LaneType_CITY_DRIVING)
      continue;

    APbPolyline* pl = new APbPolyline;
    for (auto& point :
         pb_lane.central_curve().segment(0).line_segment().point()) {
      APbVector3d* p = pl->add_points();
      p->set_x(point.x());
      p->set_y(point.y());
      p->set_z(point.z());
    }
    // AINFO << "X: " << pl->points().begin()->x() << " Y: " <<
    // pl->points().end()->x();
    auto lane_direction =
        pl->points(0).x() - pl->points(pl->points_size() - 1).x();
    // auto lane_direction =
    // pb_lane.central_curve().segment(0).line_segment().point().begin()->x() -
    // pb_lane.central_curve().segment(0).line_segment().point().end()->x();
    lane_cnt++;
    lane.set_id(lane_cnt);
    lane.set_allocated_polyline(pl);
    lane.set_turn(static_cast<APbLaneTurn>(pb_lane.turn()));
    lane.set_type(APbLaneType::Lane_Type_STREET);
    lane.set_category(APbLaneCategory::Lane_Category_MOTORWAY);
    // lane.set_road_id();
    // lane.set_right_direction(1);
    // lane.set_lane_number();
    lane.set_max_speed(
        static_cast<int32_t>(pb_lane.speed_limit() * 3.6));  // in km / hour
    lane.set_min_speed(0);                                   // in km / hour

    // lane.set_height();

    APbPolyline* polyline_bl = new APbPolyline;
    for (auto& seg : pb_lane.left_boundary().curve().segment()) {
      for (auto& point : seg.line_segment().point()) {
        APbVector3d* p = polyline_bl->add_points();
        p->set_x(point.x());
        p->set_y(point.y());
        p->set_z(point.z());
      }
    }
    auto left_boundary =
        polyline_bl->points(0).x() -
        polyline_bl->points(polyline_bl->points_size() - 1).x();

    lane.set_left_direction(lane_direction * left_boundary >
                            0);  // same direction: true
    boundary_cnt++;
    boundary_left.set_id(boundary_cnt);
    boundary_left.set_allocated_polyline(polyline_bl);
    switch (pb_lane.left_boundary().boundary_type(0).types(0)) {
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_CURB:
        boundary_left.set_property(
            APbBoundaryProperty::Boundary_Property_PHYSICAL_BLOCK);
        break;
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_DOTTED_WHITE:
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_DOTTED_YELLOW:
        boundary_left.set_property(
            APbBoundaryProperty::Boundary_Property_LEGAL_PASS);
        break;
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_SOLID_WHITE:
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_SOLID_YELLOW:
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_DOUBLE_YELLOW:
        boundary_left.set_property(
            APbBoundaryProperty::Boundary_Property_LEGAL_NO_PASS);
        break;
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_UNKNOWN:
        boundary_left.set_property(
            APbBoundaryProperty::Boundary_Property_PROPERTY_UNKNOWN);
        break;

      default:
        break;
    }
    boundary_left.set_color(APbBoundaryColor::Boundary_Color_WHITE);
    if (pb_lane.left_boundary().virtual_()) {
      boundary_left.set_type(APbBoundaryType::Boundary_Type_VIRTUAL);
    } else {
      boundary_left.set_type(APbBoundaryType::Boundary_Type_REAL);
    }
    lane.set_left_boundary_id(boundary_cnt);

    APbPolyline* polyline_br = new APbPolyline;
    for (auto& seg : pb_lane.right_boundary().curve().segment()) {
      for (auto& point : seg.line_segment().point()) {
        APbVector3d* p = polyline_br->add_points();
        p->set_x(point.x());
        p->set_y(point.y());
        p->set_z(point.z());
      }
    }
    auto right_boundary =
        polyline_br->points(0).x() -
        polyline_br->points(polyline_br->points_size() - 1).x();
    // auto right_boundary =
    // pb_lane.right_boundary().curve().segment(0).line_segment().point().begin()->x()
    // -
    // pb_lane.right_boundary().curve().segment(0).line_segment().point().end()->x();
    lane.set_right_direction(lane_direction * right_boundary >
                             0);  // same direction: true
    boundary_cnt++;
    boundary_right.set_id(boundary_cnt);
    boundary_right.set_allocated_polyline(polyline_br);
    switch (pb_lane.right_boundary().boundary_type(0).types(0)) {
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_CURB:
        boundary_right.set_property(
            APbBoundaryProperty::Boundary_Property_PHYSICAL_BLOCK);
        break;
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_DOTTED_WHITE:
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_DOTTED_YELLOW:
        boundary_right.set_property(
            APbBoundaryProperty::Boundary_Property_LEGAL_PASS);
        break;
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_SOLID_WHITE:
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_SOLID_YELLOW:
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_DOUBLE_YELLOW:
        boundary_right.set_property(
            APbBoundaryProperty::Boundary_Property_LEGAL_NO_PASS);
        break;
      case PbLaneBoundaryTypeType::LaneBoundaryType_Type_UNKNOWN:
        boundary_right.set_property(
            APbBoundaryProperty::Boundary_Property_PROPERTY_UNKNOWN);
        break;

      default:
        break;
    }
    boundary_right.set_color(APbBoundaryColor::Boundary_Color_WHITE);
    if (pb_lane.right_boundary().virtual_()) {
      boundary_right.set_type(APbBoundaryType::Boundary_Type_VIRTUAL);
    } else {
      boundary_right.set_type(APbBoundaryType::Boundary_Type_REAL);
    }
    lane.set_right_boundary_id(boundary_cnt);
    if (!lane.left_direction() || !lane.right_direction()) {
      lane.set_left_boundary_id(boundary_cnt);
      lane.set_right_boundary_id(boundary_cnt - 1);
      auto tmp_direction = lane.left_direction();
      lane.set_left_direction(lane.right_direction());
      lane.set_right_direction(tmp_direction);
    }
    if (lane_cnt == 20130) {
      AINFO << "Lane direction: " << lane_direction
            << " Left direction: " << left_boundary
            << " Right direction: " << right_boundary;
    }

    pb_map.mutable_lanes()->insert({lane.id(), lane});
    pb_map.mutable_boundaries()->insert({boundary_left.id(), boundary_left});
    pb_map.mutable_boundaries()->insert({boundary_right.id(), boundary_right});
  }

  // for (auto& crosswalk_pair : proto_data_.pb_crosswalks) {
  //   *(pb_map->add_crosswalk()) = crosswalk_pair.second;
  // }
  // for (auto& parking_space_pair : proto_data_.pb_parking_spaces) {
  //   *(pb_map->add_parking_space()) = parking_space_pair.second;
  // }
  // for (auto& clear_area_pair : proto_data_.pb_clear_areas) {
  //   *(pb_map->add_clear_area()) = clear_area_pair.second;
  // }
  // for (auto& speed_bump_pair : proto_data_.pb_speed_bumps) {
  //   *(pb_map->add_speed_bump()) = speed_bump_pair.second;
  // }
  // for (auto& signal_pair : proto_data_.pb_signals) {
  //   *(pb_map->add_signal()) = signal_pair.second;
  // }
  // for (auto& stop_sign_pair : proto_data_.pb_stop_signs) {
  //   *(pb_map->add_stop_sign()) = stop_sign_pair.second;
  // }
  // for (auto& yield_sign_pair : proto_data_.pb_yield_signs) {
  //   *(pb_map->add_yield()) = yield_sign_pair.second;
  // }
  for (auto& junction_pair : proto_data_.pb_junctions) {
    auto pb_junc = junction_pair.second;

    APbJunction junc;
    APbPolygon* polygon = new APbPolygon;
    for (auto& point : pb_junc.polygon().point()) {
      APbVector3d* p = polygon->add_points();
      p->set_x(point.x());
      p->set_y(point.y());
      p->set_z(point.z());
    }
    junction_cnt++;
    junc.set_id(junction_cnt);
    junc.set_allocated_polygon(polygon);

    pb_map.mutable_junctions()->insert({junc.id(), junc});
  }
  // for (auto& overlap_pair : proto_data_.pb_overlaps) {
  //   *(pb_map->add_overlap()) = overlap_pair.second;
  // }

  AINFO << "hdmap statistics: lanes-" << pb_map.mutable_lanes()->size()
        << " boundary-" << pb_map.mutable_boundaries()->size() << " junction-"
        << pb_map.mutable_junctions()->size();
}

void ProtoOrganizer::OutputData(apollo::hdmap::Map* pb_map) {
  for (auto& road_pair : proto_data_.pb_roads) {
    *(pb_map->add_road()) = road_pair.second;
  }
  for (auto& lane_pair : proto_data_.pb_lanes) {
    *(pb_map->add_lane()) = lane_pair.second;
  }
  for (auto& crosswalk_pair : proto_data_.pb_crosswalks) {
    *(pb_map->add_crosswalk()) = crosswalk_pair.second;
  }
  for (auto& parking_space_pair : proto_data_.pb_parking_spaces) {
    *(pb_map->add_parking_space()) = parking_space_pair.second;
  }
  for (auto& clear_area_pair : proto_data_.pb_clear_areas) {
    *(pb_map->add_clear_area()) = clear_area_pair.second;
  }
  for (auto& speed_bump_pair : proto_data_.pb_speed_bumps) {
    *(pb_map->add_speed_bump()) = speed_bump_pair.second;
  }
  for (auto& signal_pair : proto_data_.pb_signals) {
    *(pb_map->add_signal()) = signal_pair.second;
  }
  for (auto& stop_sign_pair : proto_data_.pb_stop_signs) {
    *(pb_map->add_stop_sign()) = stop_sign_pair.second;
  }
  for (auto& yield_sign_pair : proto_data_.pb_yield_signs) {
    *(pb_map->add_yield()) = yield_sign_pair.second;
  }
  for (auto& junction_pair : proto_data_.pb_junctions) {
    *(pb_map->add_junction()) = junction_pair.second;
  }
  for (auto& overlap_pair : proto_data_.pb_overlaps) {
    *(pb_map->add_overlap()) = overlap_pair.second;
  }

  AINFO << "hdmap statistics: roads-" << proto_data_.pb_roads.size()
        << ",lanes-" << proto_data_.pb_lanes.size() << ",crosswalks-"
        << proto_data_.pb_crosswalks.size() << ",parking spaces-"
        << proto_data_.pb_parking_spaces.size() << ",clear areas-"
        << proto_data_.pb_clear_areas.size() << ",speed bumps-"
        << proto_data_.pb_speed_bumps.size() << ",signals-"
        << proto_data_.pb_signals.size() << ",stop signs-"
        << proto_data_.pb_stop_signs.size() << ",yield signs-"
        << proto_data_.pb_yield_signs.size() << ",junctions-"
        << proto_data_.pb_junctions.size() << ",overlaps-"
        << proto_data_.pb_overlaps.size();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
