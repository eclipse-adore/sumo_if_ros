/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once

#define _USE_MATH_DEFINES
#include <libsumo/libsumo.h>
#include <math.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include <adore_ros2_msgs/msg/traffic_participant.hpp>
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

namespace adore
{
namespace sumo_if_ros
{
struct Timer
{
public:

  double tUTC_;
  Timer();
  void receive( const std_msgs::msg::Float64& msg );
};

struct ROSVehicleSet
{
public:

  std::unordered_map<int, adore_ros2_msgs::msg::TrafficParticipant> data;
  void receive( const adore_ros2_msgs::msg::TrafficParticipant& msg );
};

class SUMOTrafficToROS : public rclcpp::Node
{
public:

  SUMOTrafficToROS();

protected:

  ROSVehicleSet ros_vehicle_set;

  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher;
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr subscriber;
  std::string                                                               sumo_rosveh_prefix;
  rclcpp::TimerBase::SharedPtr                                              timer;

  std::unordered_map<std::string, int> sumo_veh_id_to_int;
  int                                  last_assigned_int_id;
  std::map<int, std::string>           replacement_ids;

  std::vector<std::string> veh_id_list;
  std::vector<std::string> sumo_to_ros_ignore_list;
  std::vector<int>         ros_to_sumo_ignore_list;

  rcl_time_point_value_t ros_time;
  rcl_time_point_value_t tROS0;
  double                 tSUMO0;
  double                 tSUMO;
  double                 step_length;

  int         utm_zone;
  std::string utm_letter;

  bool                 use_gui_;
  bool                 gui_tracking_set_;
  bool                 gui_follow_ego_;
  bool                 use_geo_conversion_;
  double               gui_zoom_;
  int                  ego_tracking_id_;
  libsumo::TraCIColor  ego_vehicle_color_;

  double      ego_start_x_;
  double      ego_start_y_;
  double      ego_start_heading_deg_;
  double      ego_start_sumo_x_;
  double      ego_start_sumo_y_;

  int         initial_traffic_count_;
  double      initial_traffic_spacing_;
  std::string initial_traffic_veh_type_;

protected:

  int  get_new_int_id();
  void remove_vehicle( std::string& id );
  void add_vehicle( std::string& id );
  void set_max_speed( std::string& id, double val );
  void set_speed( std::string& id, double val );
  void move_to_xy( std::string& id, std::string z, int a, double x, double y, double heading, int b );

  void parse_ego_start_position( const std::string& position_str );
  void spawn_initial_traffic();

public:

  void                   run_callback();
  rcl_time_point_value_t sumo_to_ros_time( double sumo_time );
  double                 ros_to_sumo_time( rcl_time_point_value_t ros_time );
  bool                   new_step();
  void                   transfer_data_sumo_to_ros();
  void                   transfer_data_ros_to_sumo();

  void init_sumo();
  void close_sumo();
};
} // namespace sumo_if_ros
} // namespace adore
