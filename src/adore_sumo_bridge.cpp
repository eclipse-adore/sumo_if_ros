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

#include "adore_sumo_traffic.hpp"
#include <adore_map/lat_long_conversions.hpp>
#include <random>
#include <sstream>
#include <stdexcept>

namespace adore
{
namespace sumo_if_ros
{

SUMOTrafficToROS::SUMOTrafficToROS() :
  Node( "adore_sumo_bridge" )
{
  sumo_rosveh_prefix   = "rosvehicle";
  last_assigned_int_id = 1000;

  timer = this->create_wall_timer( 10ms, std::bind( &SUMOTrafficToROS::run_callback, this ) );

  init_sumo();
  publisher  = this->create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants", 5 );
  subscriber = this->create_subscription<adore_ros2_msgs::msg::TrafficParticipant>( "simulated_traffic_participant", 1,
                                                                                    std::bind( &ROSVehicleSet::receive, &ros_vehicle_set,
                                                                                               std::placeholders::_1 ) );
}

// Parse "lat,lon,psi" string and populate ego start members.
// utm_zone and utm_letter are derived from the lat/lon via libsumo after SUMO is started.
void
SUMOTrafficToROS::parse_ego_start_position( const std::string& position_str )
{
  std::istringstream ss( position_str );
  std::string token;
  std::vector<double> values;
  while( std::getline( ss, token, ',' ) )
  {
    values.push_back( std::stod( token ) );
  }
  if( values.size() != 3 )
  {
    throw std::runtime_error( "ego_start_position must be 'lat,lon,psi' (3 comma-separated values)" );
  }
  const double lat = values[0];
  const double lon = values[1];
  const double psi = values[2]; // radians CCW from east

  // derive utm_zone and utm_letter from lat/lon via adore map conversion
  auto utm_coords = map::convert_lat_lon_to_utm( lat, lon );
  utm_zone   = static_cast<int>( utm_coords[2] );
  utm_letter = utm_coords[3] > 0 ? "U" : "S";

  ego_start_x_           = utm_coords[0];
  ego_start_y_           = utm_coords[1];
  // convert psi (radians CCW from east) to SUMO heading (degrees CW from north)
  ego_start_heading_deg_ = std::fmod( 90.0 - psi * 180.0 / M_PI + 360.0, 360.0 );

  // store SUMO internal xy directly to avoid UTM round-trip errors in spawn
  auto sumopos        = libsumo::Simulation::convertGeo( lon, lat, true );
  ego_start_sumo_x_   = sumopos.x;
  ego_start_sumo_y_   = sumopos.y;

  std::cout << "ego start: lat=" << lat << " lon=" << lon << " psi=" << psi
            << " -> utm=(" << ego_start_x_ << ", " << ego_start_y_ << ")"
            << " sumo=(" << ego_start_sumo_x_ << ", " << ego_start_sumo_y_ << ")"
            << " zone=" << utm_zone << utm_letter
            << " sumo_heading=" << ego_start_heading_deg_ << std::endl;
}

int
SUMOTrafficToROS::get_new_int_id()
{
  if( last_assigned_int_id > std::numeric_limits<int>::max() - 2 )
  {
    last_assigned_int_id = 1000;
  }
  return ++last_assigned_int_id;
}

void
SUMOTrafficToROS::remove_vehicle( std::string& id )
{
  try
  {
    libsumo::Vehicle::remove( id );
  }
  catch( ... )
  {
    std::cout << "Error: removal of sumo vehicle failed" << std::endl;
  }
}

void
SUMOTrafficToROS::add_vehicle( std::string& id )
{
  try
  {
    libsumo::Vehicle::add( id, "" );
  }
  catch( ... )
  {
    std::cout << "Error: adding a sumo vehicle failed" << std::endl;
  }
}

void
SUMOTrafficToROS::set_max_speed( std::string& id, double val )
{
  try
  {
    libsumo::Vehicle::setMaxSpeed( id, val );
  }
  catch( ... )
  {
    std::cout << "Error: setMaxSpeed for sumo vehicle failed" << std::endl;
  }
}

void
SUMOTrafficToROS::set_speed( std::string& id, double val )
{
  try
  {
    libsumo::Vehicle::setSpeed( id, val );
  }
  catch( ... )
  {
    std::cout << "Error: setSpeed for sumo vehicle failed" << std::endl;
  }
}

void
SUMOTrafficToROS::move_to_xy( std::string& id, std::string z, int a, double x, double y, double heading, int b )
{
  try
  {
    libsumo::Vehicle::moveToXY( id, z, a, x, y, heading, b );
  }
  catch( const std::exception& e )
  {
    std::cout << "Error: moveToXY for sumo vehicle " << id
              << " at (" << x << ", " << y << ") heading " << heading
              << " keepRoute=" << b << ": " << e.what() << std::endl;
  }
}

void
SUMOTrafficToROS::run_callback()
{
  if( new_step() )
  {
    transfer_data_sumo_to_ros();
    transfer_data_ros_to_sumo();

    // lock GUI onto ego vehicle on the first step it appears in the simulation,
    // deferred from add time because trackVehicle requires the vehicle to have
    // been processed by at least one simulation step
    if( !gui_tracking_set_ )
    {
      std::string ego_sumoid = sumo_rosveh_prefix + std::to_string( ego_tracking_id_ );
      if( std::find( veh_id_list.begin(), veh_id_list.end(), ego_sumoid ) != veh_id_list.end() )
      {
        try
        {
          libsumo::Vehicle::setColor( ego_sumoid, ego_vehicle_color_ );
        }
        catch( const std::exception& e )
        {
          std::cout << "setColor failed for " << ego_sumoid << ": " << e.what() << std::endl;
        }
        if( use_gui_ )
        {
          try
          {
            if( gui_follow_ego_ )
            {
              libsumo::GUI::trackVehicle( "View #0", ego_sumoid );
            }
            libsumo::GUI::setZoom( "View #0", gui_zoom_ );
          }
          catch( const std::exception& e )
          {
            std::cout << "GUI tracking failed: " << e.what() << std::endl;
          }
        }
        gui_tracking_set_ = true;
        std::cout << "ego vehicle configured: " << ego_sumoid << std::endl;
      }
    }
  }
}

rcl_time_point_value_t
SUMOTrafficToROS::sumo_to_ros_time( double sumo_time )
{
  rcl_time_point_value_t result;
  result = sumo_time * 1e9;
  return result;
}

double
SUMOTrafficToROS::ros_to_sumo_time( rcl_time_point_value_t ros_time )
{
  double result;
  result = (double) ros_time * 1e-9;
  return result;
}

bool
SUMOTrafficToROS::new_step()
{
  // returns whether sumo vehicle list is updated
  // synchronize SUMO:
  rcl_time_point_value_t new_ros_time             = this->get_clock()->now().nanoseconds();
  double                 new_target_time_for_sumo = ros_to_sumo_time( new_ros_time - tROS0 ) + tSUMO0;
  bool                   updated                  = false; // flag indicates whether sumo is actually updated
  //std::cout << "new_ros_time: " << new_ros_time << " new_target_sumo: " << new_target_time_for_sumo<< " tSUMO0: " << tSUMO0  << " tROS0: " << tROS0 <<std::endl<<std::endl;
  while( new_target_time_for_sumo - tSUMO > 0.9 * step_length )
  {
    libsumo::Simulation::step();
    double tSUMO_new = libsumo::Simulation::getTime();
    if( tSUMO_new <= tSUMO )
    {
      return false;
    }
    tSUMO   = tSUMO_new;
    updated = true;
    //std::cout << "tSUMO_new: " << tSUMO_new << std::endl<<std::endl;
  }
  if( updated )
  {
    // get vehicles from sumo
    veh_id_list = libsumo::Vehicle::getIDList();
    return true;
  }
  return false;
}

void
SUMOTrafficToROS::transfer_data_sumo_to_ros()
{
  // traffic participant information
  if( veh_id_list.size() > 0 )
  {
    adore_ros2_msgs::msg::TrafficParticipantSet tpset;
    for( auto& id : veh_id_list )
    { // iterate through sumo vehicles
      if( id.find( sumo_rosveh_prefix ) == std::string::npos )
      { // skip vehicles managed in ros
        // id translation -> match sumo vehicle to intid
        int  intid         = 0;
        auto idtranslation = sumo_veh_id_to_int.find( id );
        if( idtranslation == sumo_veh_id_to_int.end() )
        {
          intid = get_new_int_id();
          sumo_veh_id_to_int.emplace( id, intid );
        }
        else
        {
          intid = idtranslation->second;
        }
        // additional feature: skip vehicle if it is on ignore list
        if( std::find( sumo_to_ros_ignore_list.begin(), sumo_to_ros_ignore_list.end(), id ) != sumo_to_ros_ignore_list.end() )
        {
          continue;
        }

        // get vehicle data from sumo, convert it to ros message
        try
        {
          libsumo::TraCIPosition tracipos  = libsumo::Vehicle::getPosition( id );
          double                 angle_deg = libsumo::Vehicle::getAngle( id );
          double                 heading   = M_PI * 0.5 - angle_deg / 180.0 * M_PI;
          double                 v         = libsumo::Vehicle::getSpeed( id );
          std::string            type      = libsumo::Vehicle::getTypeID( id );
          double                 L         = libsumo::Vehicle::getLength( id );
          double                 w         = libsumo::Vehicle::getWidth( id );
          double                 H         = libsumo::Vehicle::getHeight( id );
          double                 v_lat     = libsumo::Vehicle::getLateralSpeed( id );

          // SUMO getPosition returns the front bumper; offset back by half the vehicle
          // length along the heading to get the centre
          double angle_rad = angle_deg * M_PI / 180.0;
          double centre_x  = tracipos.x - std::sin( angle_rad ) * L * 0.5;
          double centre_y  = tracipos.y - std::cos( angle_rad ) * L * 0.5;

          // ros message for single traffic participant
          adore_ros2_msgs::msg::TrafficParticipantDetection tp;
          tp.participant_data.tracking_id                     = intid;
          tp.participant_data.v2x_station_id                  = intid;
          tp.participant_data.motion_state.time               = tSUMO - tSUMO0;
          tp.participant_data.classification.type_id          = adore_ros2_msgs::msg::TrafficClassification::CAR;
          tp.participant_data.physical_parameters.body_height = H;
          tp.participant_data.physical_parameters.body_length = L;
          tp.participant_data.physical_parameters.body_width  = w;
          double pos_x, pos_y;
          if( use_geo_conversion_ )
          {
            auto geopos = libsumo::Simulation::convertGeo( centre_x, centre_y, false );
            auto utm    = map::convert_lat_lon_to_utm( geopos.y, geopos.x );
            pos_x = utm[0] - ego_start_x_;
            pos_y = utm[1] - ego_start_y_;
          }
          else
          {
            pos_x = centre_x;
            pos_y = centre_y;
          }
          tp.participant_data.motion_state.x                  = pos_x;
          tp.participant_data.motion_state.y                  = pos_y;
          tp.participant_data.motion_state.z                  = 0;
          tp.participant_data.motion_state.yaw_angle          = heading;
          tp.participant_data.motion_state.vx                 = v;
          tp.participant_data.motion_state.vy                 = v_lat;
          tp.participant_data.motion_state.header.frame_id    = "world";
          tp.participant_data.motion_state.header.stamp       = this->get_clock()->now();
          tpset.data.push_back( tp );
        }
        catch( ... )
        {
          std::cout << "Error: failed to get information with libsumo::Vehicle" << std::endl;
        }
      }
    }

    tpset.header.frame_id = "world";
    tpset.header.stamp    = this->get_clock()->now();
    if( publisher )
      publisher->publish( tpset );
  }
}

void
SUMOTrafficToROS::transfer_data_ros_to_sumo()
{
  // update sumo with new information from ros
  for( auto pair : ros_vehicle_set.data )
  {
    auto msg = pair.second;
    // additional feature: skip vehicles on ignore list
    if( std::find( ros_to_sumo_ignore_list.begin(), ros_to_sumo_ignore_list.end(), msg.tracking_id ) != ros_to_sumo_ignore_list.end() )
    {
      continue;
    }
    std::string sumoid;
    // get sumo id of the ros vehicle
    auto replacement_id = replacement_ids.find( msg.tracking_id );
    if( replacement_id == replacement_ids.end() )
    {
      std::stringstream ss;
      ss << sumo_rosveh_prefix << msg.tracking_id;
      sumoid = ss.str();
    }
    else
    {
      sumoid = replacement_id->second;
    }

    // convert heading: ROS yaw (radians CCW from east) -> SUMO (degrees CW from north)
    const double heading      = msg.motion_state.yaw_angle;
    const double sumo_heading = std::fmod( 90.0 - heading * 180.0 / M_PI + 360.0, 360.0 );

    double raw_x, raw_y;
    if( use_geo_conversion_ )
    {
      auto position_lat_lon = map::convert_utm_to_lat_lon( msg.motion_state.x, msg.motion_state.y, utm_zone, utm_letter );
      auto sumopos          = libsumo::Simulation::convertGeo( position_lat_lon[1], position_lat_lon[0], true );
      raw_x = sumopos.x;
      raw_y = sumopos.y;
    }
    else
    {
      raw_x = msg.motion_state.x;
      raw_y = msg.motion_state.y;
    }

    // ROS position is vehicle centre; SUMO moveToXY expects the front bumper,
    // so offset forward by half the default vehicle length (2.5m) along the heading
    const double half_length = 2.5;
    const double heading_rad = sumo_heading * M_PI / 180.0;
    const double front_x     = raw_x + std::sin( heading_rad ) * half_length;
    const double front_y     = raw_y + std::cos( heading_rad ) * half_length;

    if( std::find( veh_id_list.begin(), veh_id_list.end(), sumoid ) == veh_id_list.end() )
    {
      add_vehicle( sumoid );
      set_max_speed( sumoid, 100.0 );
      // keepRoute=6: place on nearest edge ignoring heading and routing,
      // establishes a valid edge context so subsequent keepRoute=3 calls succeed
      try
      {
        libsumo::Vehicle::moveToXY( sumoid, "", 0, front_x, front_y, sumo_heading, 6 );
      }
      catch( const std::exception& e )
      {
        std::cout << "initial placement failed for " << sumoid << ": " << e.what() << std::endl;
      }
    }

    // keepRoute=3: stay on current route but reroute if needed, handles roundabout edge transitions
    // see https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#move_to_xy_0xb4
    try
    {
      libsumo::Vehicle::moveToXY( sumoid, "", 0, front_x, front_y, sumo_heading, 3 );
    }
    catch( const std::exception& e )
    {
      std::cout << "moveToXY keepRoute=3 failed for " << sumoid
                << " at (" << front_x << ", " << front_y << ") heading " << sumo_heading
                << ": " << e.what() << " -- falling back to keepRoute=2" << std::endl;
      move_to_xy( sumoid, "", 0, front_x, front_y, sumo_heading, 2 );
    }
    set_speed( sumoid, msg.motion_state.vx );
  }
}

// Walk along the road network by a given distance from a start position,
// following successor lanes across lane boundaries.
// Returns the edge, lane position, xy, and heading at the target distance.
static bool
walk_along_network( const std::string& start_lane_id, double start_pos, double walk_distance,
                    std::string& out_edge_id, double& out_lane_pos,
                    double& out_x, double& out_y, double& out_heading_deg )
{
  std::string current_lane = start_lane_id;
  double      remaining    = walk_distance;

  for( int depth = 0; depth < 20; ++depth )
  {
    try
    {
      auto   pts      = libsumo::Lane::getShape( current_lane ).value;
      double lane_len = libsumo::Lane::getLength( current_lane );
      double target   = start_pos + remaining;

      if( target <= lane_len - 1.0 )
      {
        // target is within this lane
        double travelled = 0.0;
        for( size_t i = 0; i + 1 < pts.size(); ++i )
        {
          double seg_dx  = pts[i + 1].x - pts[i].x;
          double seg_dy  = pts[i + 1].y - pts[i].y;
          double seg_len = std::sqrt( seg_dx * seg_dx + seg_dy * seg_dy );
          if( travelled + seg_len >= target )
          {
            double t        = ( target - travelled ) / seg_len;
            out_x           = pts[i].x + t * seg_dx;
            out_y           = pts[i].y + t * seg_dy;
            out_heading_deg = std::fmod( 90.0 - std::atan2( seg_dy, seg_dx ) * 180.0 / M_PI + 360.0, 360.0 );
            out_edge_id     = current_lane.substr( 0, current_lane.rfind( '_' ) );
            out_lane_pos    = target;
            return true;
          }
          travelled += seg_len;
        }
      }

      // need to continue onto a successor lane
      remaining -= ( lane_len - start_pos );
      start_pos  = 0.0;

      auto links = libsumo::Lane::getLinks( current_lane );
      if( links.empty() ) return false;
      current_lane = links[0].approachedLane;
    }
    catch( ... ) { return false; }
  }
  return false;
}

void
SUMOTrafficToROS::spawn_initial_traffic()
{
  if( initial_traffic_count_ <= 0 )
  {
    return;
  }

  // find start edge and lane via convertRoad -- no probe vehicles or sim steps needed
  std::string start_lane_id;
  double      start_lane_pos = 0.0;
  std::string start_edge_id;

  try
  {
    auto start_road = libsumo::Simulation::convertRoad( ego_start_sumo_x_, ego_start_sumo_y_, false );
    start_edge_id   = start_road.edgeID;
    start_lane_id   = start_edge_id + "_0";
    start_lane_pos  = start_road.pos;
    std::cout << "spawn start edge: " << start_edge_id << " pos=" << start_lane_pos << std::endl;
  }
  catch( const std::exception& e )
  {
    std::cout << "convertRoad for start failed: " << e.what() << std::endl;
  }

  // walk the routing graph forward from the start edge to build a long route,
  // using Lane::getLinks to find successor edges with speed-weighted random selection
  auto build_forward_route = []( const std::string& from_edge, int min_edges, unsigned int seed ) -> std::vector<std::string>
  {
    std::mt19937 rng( seed );
    std::vector<std::string> route;
    std::string              current = from_edge;
    std::unordered_set<std::string> visited;
    while( (int)route.size() < min_edges )
    {
      if( visited.count( current ) ) break;
      visited.insert( current );
      route.push_back( current );
      try
      {
        std::string lane_id = current + "_0";
        auto        links   = libsumo::Lane::getLinks( lane_id );

        // collect candidates with their speed as weight
        std::vector<std::string> candidates;
        std::vector<double>      weights;
        for( auto& link : links )
        {
          std::string succ_lane = link.approachedLane;
          std::string succ_edge = succ_lane.substr( 0, succ_lane.rfind( '_' ) );
          if( succ_edge.empty() || visited.count( succ_edge ) ) continue;
          double spd = 1.0;
          try { spd = libsumo::Lane::getMaxSpeed( succ_lane ); } catch( ... ) {}
          candidates.push_back( succ_edge );
          weights.push_back( spd );
        }
        if( candidates.empty() ) break;

        // weighted random selection -- higher speed roads more likely but not certain
        std::discrete_distribution<size_t> dist( weights.begin(), weights.end() );
        current = candidates[ dist( rng ) ];
      }
      catch( ... ) { break; }
    }
    return route;
  };

  if( start_edge_id.empty() )
  {
    std::cout << "spawn_initial_traffic: no start edge, skipping" << std::endl;
    return;
  }

  // build a single long route from the ego start edge that all spawned vehicles share,
  // so they are placed on the ego's actual forward path rather than random junctions
  auto ego_route = build_forward_route( start_edge_id, 100,
                                        static_cast<unsigned int>( std::hash<std::string>{}( "ego_spawn_route" ) ) );
  if( ego_route.size() < 2 )
  {
    std::cout << "spawn_initial_traffic: could not build ego forward route, skipping" << std::endl;
    return;
  }

  // walk along the ego route edges to find spawn positions at each spacing interval
  // by following the route sequence rather than arbitrary lane links
  auto walk_along_route = [&]( const std::vector<std::string>& route, double start_pos, double target_dist,
                                std::string& out_edge, double& out_lane_pos,
                                double& out_x, double& out_y, double& out_heading ) -> bool
  {
    double remaining = target_dist;
    for( const auto& edge : route )
    {
      std::string lane_id = edge + "_0";
      try
      {
        double lane_len = libsumo::Lane::getLength( lane_id );
        double avail    = lane_len - start_pos - 1.0;
        if( avail <= 0.0 ) { start_pos = 0.0; continue; }
        if( remaining <= avail )
        {
          double target = start_pos + remaining;
          auto   pts    = libsumo::Lane::getShape( lane_id ).value;
          double travelled = 0.0;
          for( size_t i = 0; i + 1 < pts.size(); ++i )
          {
            double dx  = pts[i+1].x - pts[i].x;
            double dy  = pts[i+1].y - pts[i].y;
            double seg = std::sqrt( dx*dx + dy*dy );
            if( travelled + seg >= target )
            {
              double t   = ( target - travelled ) / seg;
              out_x      = pts[i].x + t * dx;
              out_y      = pts[i].y + t * dy;
              out_heading = std::fmod( 90.0 - std::atan2( dy, dx ) * 180.0 / M_PI + 360.0, 360.0 );
              out_edge    = edge;
              out_lane_pos = target;
              return true;
            }
            travelled += seg;
          }
        }
        remaining -= avail;
        start_pos = 0.0;
      }
      catch( ... ) { start_pos = 0.0; }
    }
    return false;
  };

  for( int i = 0; i < initial_traffic_count_; ++i )
  {
    std::string  id       = "sumo_spawned_" + std::to_string( i );
    const double distance = initial_traffic_spacing_ * ( i + 1 );
    double spawn_x        = ego_start_sumo_x_;
    double spawn_y        = ego_start_sumo_y_;
    double spawn_heading  = ego_start_heading_deg_;
    std::string spawn_edge_id;
    double      spawn_lane_pos = 0.0;

    if( !walk_along_route( ego_route, start_lane_pos, distance,
                           spawn_edge_id, spawn_lane_pos,
                           spawn_x, spawn_y, spawn_heading ) )
    {
      std::cout << "skipping spawn of " << id << ": walk_along_route failed at distance " << distance << std::endl;
      continue;
    }

    if( spawn_edge_id.empty() )
    {
      std::cout << "skipping spawn of " << id << ": could not resolve spawn edge" << std::endl;
      continue;
    }

    try
    {
      auto route_edges = build_forward_route( spawn_edge_id, 50,
                             static_cast<unsigned int>( std::hash<std::string>{}( id ) ) );
      if( route_edges.size() < 2 )
      {
        std::cout << "skipping spawn of " << id << ": could not build forward route from " << spawn_edge_id << std::endl;
        continue;
      }

      std::string route_id = id + "_route";
      libsumo::Route::add( route_id, route_edges );
      libsumo::Vehicle::add( id, route_id, initial_traffic_veh_type_, "now", "best",
                             "random_free", "0" );
      try
      {
        libsumo::Vehicle::moveToXY( id, spawn_edge_id, 0, spawn_x, spawn_y, spawn_heading, 2 );
      }
      catch( const std::exception& e )
      {
        std::cout << "moveToXY failed for " << id << ": " << e.what() << std::endl;
      }
      std::cout << "spawned traffic vehicle " << id
                << " at (" << spawn_x << ", " << spawn_y << ")"
                << " edge=" << spawn_edge_id << " pos=" << spawn_lane_pos
                << " route: " << route_edges.front() << " -> " << route_edges.back()
                << " (" << route_edges.size() << " edges)" << std::endl;
    }
    catch( const std::exception& e )
    {
      std::cout << "failed to spawn " << id << ": " << e.what() << std::endl;
    }
  }
}

void
SUMOTrafficToROS::init_sumo()
{
  int port    = -1;
  step_length = 0.05;
  std::string cfg_file;
  declare_parameter( "sumo_config_file", "" );
  get_parameter( "sumo_config_file", cfg_file );
  declare_parameter( "sumo_step_length", 0.05 );
  get_parameter( "sumo_step_length", step_length );
  if( cfg_file.empty() )
  {
    throw std::runtime_error( "Error: No config file for sumo provided." );
  }
  use_gui_ = false;
  declare_parameter( "use_gui", false );
  get_parameter( "use_gui", use_gui_ );
  std::string gui_settings_file;
  declare_parameter( "gui_settings_file", std::string( "" ) );
  get_parameter( "gui_settings_file", gui_settings_file );
  ego_tracking_id_ = 0;
  declare_parameter( "ego_tracking_id", 0 );
  get_parameter( "ego_tracking_id", ego_tracking_id_ );
  gui_zoom_ = 5000.0;
  declare_parameter( "gui_zoom", 5000.0 );
  get_parameter( "gui_zoom", gui_zoom_ );
  gui_follow_ego_ = true;
  declare_parameter( "gui_follow_ego", true );
  get_parameter( "gui_follow_ego", gui_follow_ego_ );
  gui_tracking_set_      = false;
  // ego vehicle color as "R,G,B" or "R,G,B,A" with values 0-255; default bright yellow
  ego_vehicle_color_ = { 255, 255, 0, 255 };
  std::string ego_vehicle_color_str = "255,255,0";
  declare_parameter( "ego_vehicle_color", std::string( "255,255,0" ) );
  get_parameter( "ego_vehicle_color", ego_vehicle_color_str );
  {
    std::istringstream ss( ego_vehicle_color_str );
    std::string token;
    std::vector<int> c;
    while( std::getline( ss, token, ',' ) )
    {
      c.push_back( std::stoi( token ) );
    }
    if( c.size() >= 3 )
    {
      ego_vehicle_color_.r = static_cast<uint8_t>( c[0] );
      ego_vehicle_color_.g = static_cast<uint8_t>( c[1] );
      ego_vehicle_color_.b = static_cast<uint8_t>( c[2] );
      ego_vehicle_color_.a = c.size() >= 4 ? static_cast<uint8_t>( c[3] ) : 255;
    }
  }
  use_geo_conversion_ = true;
  declare_parameter( "use_geo_conversion", true );
  get_parameter( "use_geo_conversion", use_geo_conversion_ );
  initial_traffic_count_ = 0;
  initial_traffic_spacing_ = 20.0;
  initial_traffic_veh_type_ = "DEFAULT_VEHTYPE";
  declare_parameter( "initial_traffic_count", 0 );
  declare_parameter( "initial_traffic_spacing", 20.0 );
  declare_parameter( "initial_traffic_veh_type", std::string( "DEFAULT_VEHTYPE" ) );
  get_parameter( "initial_traffic_count", initial_traffic_count_ );
  get_parameter( "initial_traffic_spacing", initial_traffic_spacing_ );
  get_parameter( "initial_traffic_veh_type", initial_traffic_veh_type_ );
  std::vector<std::string> sumoargs;
  sumoargs.push_back( use_gui_ ? "sumo-gui" : "sumo" );
  sumoargs.push_back( "-c" );
  sumoargs.push_back( cfg_file );
  sumoargs.push_back( "--step-length" );
  sumoargs.push_back( std::to_string( step_length ) );
  if( use_gui_ )
  {
    sumoargs.push_back( "--start" ); // begin stepping immediately without waiting for play button
    if( !gui_settings_file.empty() )
    {
      sumoargs.push_back( "--gui-settings-file" );
      sumoargs.push_back( gui_settings_file );
    }
  }
  if( port > -1 )
  {
    sumoargs.push_back( "--remote-port" );
    sumoargs.push_back( std::to_string( port ) );
  }

  while( true )
  {
    try
    {
      std::cout << "load sumo ..." << std::flush;
      libsumo::Simulation::start( sumoargs );
      std::cout << " done." << std::endl;
      break;
    }
    catch( const std::exception& exc )
    {
      std::cout << exc.what() << std::endl;
      std::cout << "Arguments for sumo:" << std::endl;
      for( auto a : sumoargs )
      {
        std::cout << a << std::endl;
      }
      std::cout << "try again ..." << std::endl;
    }
    std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
  }
  std::cout << "started sumo with config file " << cfg_file << " and step length " << step_length << std::endl;

  // ego_start_position ("lat,lon,psi") is optional -- when absent the bridge operates
  // in local coordinate mode using explicit utm_zone/utm_letter parameters instead,
  // which is needed for synthetic scenarios without real-world georeferencing
  std::string ego_start_position_str;
  declare_parameter( "ego_start_position", std::string( "" ) );
  get_parameter( "ego_start_position", ego_start_position_str );
  if( !ego_start_position_str.empty() )
  {
    parse_ego_start_position( ego_start_position_str );
  }
  else
  {
    int         fallback_zone   = 32;
    std::string fallback_letter = "U";
    declare_parameter( "utm_zone",   32 );
    declare_parameter( "utm_letter", std::string( "U" ) );
    get_parameter( "utm_zone",   fallback_zone );
    get_parameter( "utm_letter", fallback_letter );
    utm_zone   = fallback_zone;
    utm_letter = fallback_letter;
    ego_start_sumo_x_      = 0.0;
    ego_start_sumo_y_      = 0.0;
    ego_start_heading_deg_ = 0.0;
    std::cout << "no ego_start_position set, using utm_zone=" << utm_zone
              << " utm_letter=" << utm_letter << std::endl;
  }



  // Take one step so SUMO is fully running before capturing reference times,
  // ensuring both clocks are anchored at the same actual simulation start point.
  libsumo::Simulation::step();

  tSUMO0   = libsumo::Simulation::getTime();
  tSUMO    = tSUMO0;
  tROS0    = this->get_clock()->now().nanoseconds();
  ros_time = tROS0;

  spawn_initial_traffic();
}

void
SUMOTrafficToROS::close_sumo()
{
  libsumo::Simulation::close();
}

Timer::Timer() :
  tUTC_( 0.0 )
{}

void
Timer::receive( const std_msgs::msg::Float64& msg )
{
  tUTC_ = msg.data;
}

void
ROSVehicleSet::receive( const adore_ros2_msgs::msg::TrafficParticipant& msg )
{
  if( data.find( msg.tracking_id ) == data.end() )
  {
    data.emplace( msg.tracking_id, msg );
  }
  else
  {
    data[msg.tracking_id] = msg;
  }
}

} // namespace sumo_if_ros
} // namespace adore

int
main( int argc, char** argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::sumo_if_ros::SUMOTrafficToROS>() );
  rclcpp::shutdown();
  return 0;
}
