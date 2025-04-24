/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *   Matthias Nichting
 ********************************************************************************/

#include "adore_sumo_traffic.hpp"

namespace adore {
namespace sumo_if_ros {

SUMOTrafficToROS::SUMOTrafficToROS() : Node("adore_sumo_bridge") {
  timer = this->create_wall_timer(
      10ms, std::bind(&SUMOTrafficToROS::runCallback, this));
  init_sumo();
  publisher_ =
      this->create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>(
          "traffic_participants", 5);

  subscriber_ =
      this->create_subscription<adore_ros2_msgs::msg::TrafficParticipant>(
          "vehicle_state/traffic_participant", 1,
          std::bind(&ROSVehicleSet::receive, &rosVehicleSet_,
                    std::placeholders::_1));
  sumo_rosveh_prefix_ = "rosvehicle";
  last_assigned_int_id_ = 1000;  // start sumo ids with 1001
}

int SUMOTrafficToROS::getNewIntID() {
  if (last_assigned_int_id_ > std::numeric_limits<int>::max() - 2) {
    last_assigned_int_id_ = 1000;
  }
  return ++last_assigned_int_id_;
}

void SUMOTrafficToROS::removeVehicle(std::string& id) {
  try {
    libsumo::Vehicle::remove(id);
  } catch (...) {
    std::cout << "Error: removal of sumo vehicle failed" << std::endl;
  }
}

void SUMOTrafficToROS::addVehicle(std::string& id) {
  try {
    libsumo::Vehicle::add(id, "");
  } catch (...) {
    std::cout << "Error: adding a sumo vehicle failed" << std::endl;
  }
}

void SUMOTrafficToROS::setMaxSpeed(std::string& id, double val) {
  try {
    libsumo::Vehicle::setMaxSpeed(id, val);
  } catch (...) {
    std::cout << "Error: setMaxSpeed for sumo vehicle failed" << std::endl;
  }
}

void SUMOTrafficToROS::setSpeed(std::string& id, double val) {
  try {
    libsumo::Vehicle::setSpeed(id, val);
  } catch (...) {
    std::cout << "Error: setSpeed for sumo vehicle failed" << std::endl;
  }
}

void SUMOTrafficToROS::moveToXY(std::string& id, std::string z, int a, double x,
                                double y, double heading, int b) {
  try {
    libsumo::Vehicle::moveToXY(id, z, a, x, y, heading, b);
  } catch (...) {
    std::cout << "Error: moveToXY for sumo vehicle failed" << std::endl;
  }
}

void SUMOTrafficToROS::runCallback() {
  if (newStep()) {
    transferDataSumoToRos();
    transferDataRosToSumo();
  }
}

rcl_time_point_value_t SUMOTrafficToROS::sumo_to_ros_time(double sumo_time) {
  rcl_time_point_value_t result;
  result = sumo_time * 1e9;
  return result;
}

double SUMOTrafficToROS::ros_to_sumo_time(rcl_time_point_value_t ros_time) {
  double result;
  result = (double)ros_time * 1e-9;
  return result;
}

bool SUMOTrafficToROS::newStep() {
  // returns whether sumo vehicle list is updated
  // synchronize SUMO:
  rcl_time_point_value_t new_ros_time = this->get_clock()->now().nanoseconds();
  double new_target_time_for_sumo =
      ros_to_sumo_time(new_ros_time - tROS0) + tSUMO0;
  bool updated = false;  // flag indicates whether sumo is actually updated
  while (new_target_time_for_sumo - tSUMO > 0.9 * step_length) {
    libsumo::Simulation::step();
    double tSUMO_new = libsumo::Simulation::getTime();
    if (tSUMO_new <= tSUMO) {
      return false;
    }
    tSUMO = tSUMO_new;
    updated = true;
  }
  if (updated) {
    // get vehicles from sumo
    vehidlist_ = libsumo::Vehicle::getIDList();
    return true;
  }
  return false;
}

void SUMOTrafficToROS::transferDataSumoToRos() {
  // traffic participant information
  if (vehidlist_.size() > 0) {
    // message for set of traffic participants
    adore_ros2_msgs::msg::TrafficParticipantSet tpset;
    for (auto& id : vehidlist_) {  // iterate through sumo vehicles
      if (id.find(sumo_rosveh_prefix_) ==
          std::string::npos) {  // skip vehicles managed in ros
        // id translation -> match sumo vehicle to intid
        int intid = 0;
        auto idtranslation = sumovehid2int_.find(id);
        if (idtranslation == sumovehid2int_.end()) {
          intid = getNewIntID();
          sumovehid2int_.emplace(id, intid);
        } else {
          intid = idtranslation->second;
        }
        // additional feature: skip vehicle if it is on ignore list
        if (std::find(sumo_to_ros_ignore_list_.begin(),
                      sumo_to_ros_ignore_list_.end(),
                      id) != sumo_to_ros_ignore_list_.end()) {
          continue;
        }

        // get vehicle data from sumo, convert it to ros message
        try {
          libsumo::TraCIPosition tracipos = libsumo::Vehicle::getPosition(id);
          double heading =
              M_PI * 0.5 - libsumo::Vehicle::getAngle(id) / 180.0 * M_PI;
          double v = libsumo::Vehicle::getSpeed(id);
          std::string type = libsumo::Vehicle::getTypeID(id);
          double L = libsumo::Vehicle::getLength(id);
          double w = libsumo::Vehicle::getWidth(id);
          double H = libsumo::Vehicle::getHeight(id);
          double v_lat = libsumo::Vehicle::getLateralSpeed(id);

          // ros message for single traffic participant
          adore_ros2_msgs::msg::TrafficParticipantDetection tp;
          tp.participant_data.tracking_id = intid;
          tp.participant_data.v2x_station_id = intid;
          tp.participant_data.motion_state.time = tSUMO - tSUMO0;
          tp.participant_data.classification.type_id =
              adore_ros2_msgs::msg::TrafficClassification::CAR;
          tp.participant_data.shape.dimensions.push_back(L);
          tp.participant_data.shape.dimensions.push_back(w);
          tp.participant_data.shape.dimensions.push_back(H);
          auto geopos =
              libsumo::Simulation::convertGeo(tracipos.x, tracipos.y, false);
          tp.participant_data.motion_state.x = geopos.x;
          tp.participant_data.motion_state.y = geopos.y;
          tp.participant_data.motion_state.z = 0;
          tp.participant_data.motion_state.yaw_angle = heading;
          tp.participant_data.motion_state.vx = v;
          tp.participant_data.motion_state.vy = v_lat;
          // add the traffic participant to the set
          tpset.data.push_back(tp);
        } catch (...) {
          std::cout << "Error: failed to get information with libsumo::Vehicle"
                    << std::endl;
        }
      }
    }

    // publish the vehicle data in ros
    publisher_->publish(tpset);
  }
}

void SUMOTrafficToROS::transferDataRosToSumo() {
  // update sumo with new information from ros
  for (auto pair : rosVehicleSet_.data_) {
    auto msg = pair.second;
    // additional feature: skip vehicles on ignore list
    if (std::find(ros_to_sumo_ignore_list_.begin(),
                  ros_to_sumo_ignore_list_.end(),
                  msg.tracking_id) != ros_to_sumo_ignore_list_.end()) {
      continue;
    }
    std::string sumoid;
    // get sumo id of the ros vehicle
    auto replacement_id = replacement_ids_.find(msg.tracking_id);
    if (replacement_id == replacement_ids_.end()) {
      std::stringstream ss;
      ss << sumo_rosveh_prefix_ << msg.tracking_id;
      sumoid = ss.str();
    } else {
      sumoid = replacement_id->second;
    }
    // add vehicle in sumo if not yet existent
    if (std::find(vehidlist_.begin(), vehidlist_.end(), sumoid) ==
        vehidlist_.end()) {
      addVehicle(sumoid);
      setMaxSpeed(sumoid, 100.0);
    }

    // set speed and position of corresponding sumo vehicle accordingly
    const double heading = msg.motion_state.yaw_angle;
    auto sumopos = libsumo::Simulation::convertGeo(msg.motion_state.x,
                                                   msg.motion_state.y, true);
    // following line: keepRoute=2. see
    // https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#move_to_xy_0xb4
    // for details
    moveToXY(sumoid, "", 0, sumopos.x, sumopos.y, heading, 2);
    setSpeed(sumoid, msg.motion_state.vx);
  }
}

void SUMOTrafficToROS::init_sumo() {
  int port = -1;
  step_length = 0.05;
  std::string cfg_file;
  declare_parameter("sumo config file", "");
  get_parameter("sumo config file", cfg_file);
  declare_parameter("sumo step length", 0.01);
  get_parameter("sumo step length", step_length);
  if (cfg_file.empty()) {
    throw std::runtime_error("Error: No config file for sumo provided.");
  }
  std::vector<std::string> sumoargs;
  sumoargs.push_back("sumo");
  sumoargs.push_back("-c");
  sumoargs.push_back(cfg_file);
  sumoargs.push_back("--step-length");
  sumoargs.push_back(std::to_string(step_length));
  if (port > -1) {
    sumoargs.push_back("--remote-port");
    sumoargs.push_back(std::to_string(port));
  }

  while (true) {
    try {
      std::cout << "load sumo ..." << std::flush;
      // libsumo::Simulation::load(sumoargs);
      libsumo::Simulation::start(sumoargs);
      std::cout << " done." << std::endl;
      break;
    } catch (const std::exception& exc) {
      std::cout << exc.what() << std::endl;
      std::cout << "Arguments for sumo:" << std::endl;
      for (auto a : sumoargs) {
        std::cout << a << std::endl;
      }
      std::cout << "try again ..." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  std::cout << "started sumo with config file " << cfg_file
            << " and step length " << step_length << std::endl;
  tSUMO0 = libsumo::Simulation::getTime();
  tSUMO = tSUMO0;  // sumo time at startup
  tROS0 = this->get_clock()->now().nanoseconds();
  ros_time = tROS0;  // ros time at startup
}

void SUMOTrafficToROS::closeSumo() { libsumo::Simulation::close(); }

Timer::Timer() : tUTC_(0.0) {}
void Timer::receive(const std_msgs::msg::Float64& msg) { tUTC_ = msg.data; }

void ROSVehicleSet::receive(
    const adore_ros2_msgs::msg::TrafficParticipant& msg) {
  if (data_.find(msg.tracking_id) == data_.end()) {
    data_.emplace(msg.tracking_id, msg);
  } else {
    data_[msg.tracking_id] = msg;
  }
}

}  // namespace sumo_if_ros
}  // namespace adore

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adore::sumo_if_ros::SUMOTrafficToROS>());
  rclcpp::shutdown();
  return 0;
}