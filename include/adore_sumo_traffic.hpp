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
  void receive( const std_msgs::msg::Float64& msg ); // callback for ros time
};

struct ROSVehicleSet
{
public:

  std::unordered_map<int, adore_ros2_msgs::msg::TrafficParticipant> data; ///<- a mapping from vehicle
                                                                          ///< id to
                                                                          ///< latest message
  void receive( const adore_ros2_msgs::msg::TrafficParticipant& msg );    // callback for receiving ros vehicle data
};

class SUMOTrafficToROS : public rclcpp::Node
{
public:

  SUMOTrafficToROS();

protected:

  ROSVehicleSet ros_vehicle_set; // ros vehicle data

  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher;          // publisher for sumo traffic data
  rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr subscriber;         // subscription of dynamic vehicle state
  std::string                                                               sumo_rosveh_prefix; // prefix for sumo ros vehicles
  rclcpp::TimerBase::SharedPtr                                              timer;              // main timer

  std::unordered_map<std::string, int> sumo_veh_id_to_int;   // mapping for id of sumo vehicles
  int                                  last_assigned_int_id; // last used id
  std::map<int, std::string>           replacement_ids;      // additional feature to replace certain ids

  std::vector<std::string> veh_id_list;             // list of sumo vehicles
  std::vector<std::string> sumo_to_ros_ignore_list; // additional feature to ignore certain ids
  std::vector<int>         ros_to_sumo_ignore_list; // additional feature to ignore certain ids

  rcl_time_point_value_t ros_time;    // current ros time
  rcl_time_point_value_t tROS0;       // ros time at startup
  double                 tSUMO0;      // sumo time at startup
  double                 tSUMO;       // current sumo time
  double                 step_length; // step length in s

public:

protected:

  int  get_new_int_id();                                                                               // get new unique integer id
  void remove_vehicle( std::string& id );                                                              // remove vehicle in sumo
  void add_vehicle( std::string& id );                                                                 // add vehicle in sumo
  void set_max_speed( std::string& id, double val );                                                   // set max speed in sumo
  void set_speed( std::string& id, double val );                                                       // set speed in sumo
  void move_to_xy( std::string& id, std::string z, int a, double x, double y, double heading, int b ); // move vehicle in sumo

public:

  void                   run_callback();                                      // run function triggered by timer
  rcl_time_point_value_t sumo_to_ros_time( double sumo_time );                // conversion of sumo time to ros time
  double                 ros_to_sumo_time( rcl_time_point_value_t ros_time ); // conversion of ros time to sumo time
  bool                   new_step();                                          // perform new step in sumo
  void                   transfer_data_sumo_to_ros();                         // transfer data from sumo to ros
  void                   transfer_data_ros_to_sumo();                         // transfer data from ros to sumo

  void init_sumo();  // initialize sumo
  void close_sumo(); // close sumo
};
} // namespace sumo_if_ros
} // namespace adore