/********************************************************************************
 * Copyright (C) 2017-2022 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *   Daniel Heß - initial API and implementation
 *   Matthias Nichting
 ********************************************************************************/
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
// nim #include "adore_sumo_tls.hpp"
#include <iostream>
#include <libsumo/libsumo.h>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/float64.hpp>
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/traffic_participant.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <unordered_set>
#include <unordered_map>
using namespace std::chrono_literals;
namespace adore
{
    namespace sumo_if_ros
    {
        struct Timer
        {
          public:
            double tUTC_;
            Timer() : tUTC_(0.0)
            {
            }
            void receive(const std_msgs::msg::Float64& msg)
            {
                tUTC_ = msg.data;
            }
        };

        struct ROSVehicleSet
        {
          public:
            std::unordered_map<int, adore_ros2_msgs::msg::TrafficParticipantDetection> data_;  ///<- a mapping from vehicle
                                                                                            ///< id to
                                                                                            ///< latest message
            void receive(const adore_ros2_msgs::msg::TrafficParticipantDetection& msg)
            {
                // data_.push_back(*msg);
                if (data_.find(msg.participant_data.tracking_id) == data_.end())
                {
                    data_.emplace(msg.participant_data.tracking_id, msg);
                }
                else
                {
                    data_[msg.participant_data.tracking_id] = msg;
                }
            }
        };

        class SUMOTrafficToROS: public rclcpp::Node
        {
          public:
            SUMOTrafficToROS():Node( "adore_sumo_bridge" )
            {
                std::cout << "CTR" << std::endl;
                timer = this->create_wall_timer(10ms, std::bind(&SUMOTrafficToROS::runCallback, this));
                init_sumo();
                delta_t_ = 0;
                min_update_period_ = 0.01; // seconds
                min_tl_update_period_ = 1;
                last_tl_update_time_ = 0;
            }
            ~SUMOTrafficToROS()
            {
            }

          protected:
            ROSVehicleSet rosVehicleSet_;

            
            rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher_;
            // nim rclcpp::Publisher<dsrc_v2_mapem_pdu_descriptions_msgs::msg::MAPEM>::SharedPtr mapem_publisher_;
            // nim rclcpp::Publisher<dsrc_v2_spatem_pdu_descriptions_msgs::msg::SPATEM>::SharedPtr spatem_publisher_;
         // nim   SumoTLs2Ros sumotls2ros;
            double last_tl_update_time_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber1_;
            rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipantDetection>::SharedPtr subscriber2_;
            std::string sumo_rosveh_prefix_;
            std::string sumo_rosped_prefix_;
            rclcpp::TimerBase::SharedPtr timer;



            std::unordered_map<std::string, int> sumovehid2int_;
            std::unordered_map<std::string, int> sumopedid2int_;
            int last_assigned_int_id_;
            std::map<int, std::string> replacement_ids_;

            std::vector<std::string> vehidlist_;
            std::vector<std::string> pedidlist_;
            std::vector<std::string> sumo_to_ros_ignore_list_;
            std::vector<int> ros_to_sumo_ignore_list_;

            rcl_time_point_value_t ros_time;
            rcl_time_point_value_t tROS0;
            double tSUMO0;
            double tSUMO;
            int64_t delta_t_;
            double min_update_period_;
            double min_tl_update_period_;
            double step_length;
          public:

          protected:
            int getNewIntID()
            {
                if (last_assigned_int_id_ > std::numeric_limits<int>::max() - 2)
                {
                    last_assigned_int_id_ = 1000;
                }
                return ++last_assigned_int_id_;
            }
            void removeVehicle(std::string& id)
            {
                try
                {
                    libsumo::Vehicle::remove(id);
                }
                catch (...)
                {
                    std::cout << "Error: removal of sumo vehicle failed" << std::endl;
                }
            }
            void addVehicle(std::string& id)
            {
                try
                {
                    libsumo::Vehicle::add(id, "");
                }
                catch (...)
                {
                    std::cout << "Error: adding a sumo vehicle failed" << std::endl;
                }
            }
            void setMaxSpeed(std::string& id, double val)
            {
                try
                {
                    libsumo::Vehicle::setMaxSpeed(id, val);
                }
                catch (...)
                {
                    std::cout << "Error: setMaxSpeed for sumo vehicle failed" << std::endl;
                }
            }
            void setSpeed(std::string& id, double val)
            {
                try
                {
                    libsumo::Vehicle::setSpeed(id, val);
                }
                catch (...)
                {
                    std::cout << "Error: setSpeed for sumo vehicle failed" << std::endl;
                }
            }
            void moveToXY(std::string& id, std::string z, int a, double x, double y, double heading, int b)
            {
                try
                {
                    libsumo::Vehicle::moveToXY(id, z, a, x, y, heading, b);
                }
                catch (...)
                {
                    std::cout << "Error: moveToXY for sumo vehicle failed" << std::endl;
                }
            }

          public:
            void runCallback()
            {
                if (newStep())
                {
                    transferDataSumoToRos();
                    transferDataRosToSumo();
                }
            }
            rcl_time_point_value_t sumo_to_ros_time (double sumo_time)
            {
                rcl_time_point_value_t result;
                std::cout << "input "<< sumo_time << std::endl;
                result = sumo_time * 1e9;
                std::cout << "output "<< result << std::endl;
                return result;
            }
            double ros_to_sumo_time (rcl_time_point_value_t ros_time)
            {
                double result;
                result = (double)ros_time * 1e-9;
                return result;
            }
            bool newStep()
            {
                // synchronize SUMO:
                rcl_time_point_value_t new_ros_time = this->get_clock()->now().nanoseconds();
                double new_target_time_for_sumo = ros_to_sumo_time(new_ros_time - tROS0) + tSUMO0;
                std::cout << std::endl<<"new target time " << new_target_time_for_sumo << std::endl;
                bool updated =false;
                if (new_target_time_for_sumo > 1.0 && new_target_time_for_sumo < 1.2)
                {
                    return false;
                }
                while (new_target_time_for_sumo - tSUMO > 0.99 * step_length)
                {
                    libsumo::Simulation::step();
                    double tSUMO_new = libsumo::Simulation::getTime();
                    if (tSUMO_new <= tSUMO)
                    {
                        return false;
                    }
                    tSUMO = tSUMO_new;                
                }
                if (updated)
                {
                    // get vehicles from sumo
                    vehidlist_ = libsumo::Vehicle::getIDList();
                    pedidlist_ = libsumo::Person::getIDList();
                    return true;
                }
                return false;
            }
            void transferDataSumoToRos()
            {
                // traffic light information
        // nim         if (timer_.tUTC_ - last_tl_update_time_ > min_tl_update_period_)
       // nim          {
      // nim              auto v2x_mapem = sumotls2ros.getMAPEMFromSUMO(timer_.tUTC_);
      // nim              auto spatems = sumotls2ros.getSPATEMFromSUMO(timer_.tUTC_);
      // nim          
      // nim              // resend static mapem data
      // nim              for (auto&& mapem_item : v2x_mapem)
      // nim                  mapem_publisher_->publish(mapem_item.second);
      // nim          
      // nim              // send non-static spatem data
      // nim              for (auto&& spat : spatems)
      // nim                  spatem_publisher_->publish(spat);
                
       // nim              last_tl_update_time_ = timer_.tUTC_;
       // nim          }
                // traffic participant information
                if (vehidlist_.size() > 0 || pedidlist_.size() > 0)
                {
                    // message for set of traffic participants
                    adore_ros2_msgs::msg::TrafficParticipantSet tpset;
                    for (auto& id : vehidlist_)
                    {
                        if (id.find(sumo_rosveh_prefix_) == std::string::npos)
                        {
                            // id translation
                            int intid = 0;
                            auto idtranslation = sumovehid2int_.find(id);
                            if (idtranslation == sumovehid2int_.end())
                            {
                                intid = getNewIntID();
                                sumovehid2int_.emplace(id, intid);
                            }
                            else
                            {
                                intid = idtranslation->second;
                            }
                            if (std::find(sumo_to_ros_ignore_list_.begin(), sumo_to_ros_ignore_list_.end(), id) !=
                                sumo_to_ros_ignore_list_.end())
                            {
                                continue;
                            }
                            try
                            {
                                libsumo::TraCIPosition tracipos = libsumo::Vehicle::getPosition(id);
                                double heading = M_PI * 0.5 - libsumo::Vehicle::getAngle(id) / 180.0 * M_PI;
                                double v = libsumo::Vehicle::getSpeed(id);
                                std::string type = libsumo::Vehicle::getTypeID(id);
                                double L = libsumo::Vehicle::getLength(id);
                                double w = libsumo::Vehicle::getWidth(id);
                                double H = libsumo::Vehicle::getHeight(id);
                                double v_lat = libsumo::Vehicle::getLateralSpeed(id);
                                //nim int signals = libsumo::Vehicle::getSignals(id);  ///<- bit array! see TraciAPI:669,
                                //                                                 ///< VehicleSignal

                                // ros message for single traffic participant
                                adore_ros2_msgs::msg::TrafficParticipantDetection tp;
                                tp.participant_data.tracking_id = intid;
                              //nim  tp.data.v2x_station_id = intid;
                                tp.participant_data.motion_state.time = tSUMO - tSUMO0;
                                tp.participant_data.classification.type_id =
                                    adore_ros2_msgs::msg::TrafficClassification::CAR;  //@TODO: parse sumo type string
                                tp.participant_data.shape.dimensions.push_back(L);
                                tp.participant_data.shape.dimensions.push_back(w);
                                tp.participant_data.shape.dimensions.push_back(H);
                                auto geopos = libsumo::Simulation::convertGeo(tracipos.x, tracipos.y, false);
                                tp.participant_data.motion_state.x = geopos.x;
                                tp.participant_data.motion_state.y = geopos.y;
                                tp.participant_data.motion_state.z = 0;
                               //nim tf2::Quaternion q;
                               //nim q.setRPY(0.0, 0.0, heading);
                                tp.participant_data.motion_state.yaw_angle = heading;
                                tp.participant_data.motion_state.vx = v;
                                tp.participant_data.motion_state.vy = v_lat;
                                // add the traffic participant to the set
                                tpset.data.push_back(tp);
                            }
                            catch (...)
                            {
                                std::cout << "Error: failed to get information with libsumo::Vehicle" << std::endl;
                            }
                        }
                    }

                    for (auto& id : pedidlist_)
                    {
                        // ignore pedestrians controlled by other simulator
                        if (id.find(sumo_rosped_prefix_) == std::string::npos)
                        {
                            // id translation
                            int intid = 0;
                            auto idtranslation = sumopedid2int_.find(id);
                            if (idtranslation == sumopedid2int_.end())
                            {
                                intid = getNewIntID();
                                sumopedid2int_.emplace(id, intid);
                            }
                            else
                            {
                                intid = idtranslation->second;
                            }

                            // retrieve person data
                            libsumo::TraCIPosition tracipos = libsumo::Person::getPosition(id);
                            double heading = M_PI * 0.5 - libsumo::Person::getAngle(id) / 180.0 * M_PI;
                            double v = libsumo::Person::getSpeed(id);
                            std::string type = libsumo::Person::getTypeID(id);
                            double L = libsumo::Person::getLength(id);
                            double w = libsumo::Person::getLength(id);
                            double H = 1.75;
                            // not in libsumo?  int signals = libtraci::Person::getSignals(id); ///<- bit array!
                            // see TraciAPI:669, todo int signals = libsumo::Person::getSignals(id);  ///<- bit
                            // array! see TraciAPI:669, VehicleSignal

                            // ros message for single traffic participant
                            adore_ros2_msgs::msg::TrafficParticipantDetection tp;
                            tp.participant_data.tracking_id = intid;
                            tp.participant_data.motion_state.time = tSUMO - tSUMO0;
                            tp.participant_data.classification.type_id = adore_ros2_msgs::msg::TrafficClassification::PEDESTRIAN;
                            
                            tp.participant_data.shape.dimensions.push_back(L);
                            tp.participant_data.shape.dimensions.push_back(w);
                            tp.participant_data.shape.dimensions.push_back(H);
                            auto geopos = libsumo::Simulation::convertGeo(tracipos.x, tracipos.y, false);
                            tp.participant_data.motion_state.x = geopos.x;
                            tp.participant_data.motion_state.y = geopos.y;
                            tp.participant_data.motion_state.z = 0;
                            // nim tf2::Quaternion q;
                            // nim q.setRPY(0.0, 0.0, heading);
                            tp.participant_data.motion_state.yaw_angle = heading;
                            // add the traffic participant to the set
                            tpset.data.push_back(tp);
                        }
                    }

                    // write the message
                    publisher_->publish(tpset);
                }
            }
            void transferDataRosToSumo()
            {
                // update sumo with new information from ros
                // TODO std::vector<int> delete_from_rosvehicleset;
                for (auto pair : rosVehicleSet_.data_)
                {
                    auto msg = pair.second;
                    if (std::find(ros_to_sumo_ignore_list_.begin(), ros_to_sumo_ignore_list_.end(), msg.participant_data.tracking_id) !=
                        ros_to_sumo_ignore_list_.end())
                    {
                        continue;
                    }
                    std::string sumoid;
                    auto replacement_id = replacement_ids_.find(msg.participant_data.tracking_id);
                    if (replacement_id == replacement_ids_.end())
                    {
                        std::stringstream ss;
                        ss << sumo_rosveh_prefix_ << msg.participant_data.tracking_id;
                        sumoid = ss.str();
                    }
                    else
                    {
                        sumoid = replacement_id->second;
                    }
                    if (std::find(vehidlist_.begin(), vehidlist_.end(), sumoid) == vehidlist_.end())
                    {
                        addVehicle(sumoid);
                        setMaxSpeed(sumoid, 100.0);
                    }

                    // nim auto p = msg.data.motion_state.pose.pose;
                    // nim tf2::Quaternion q;
                    // nim q.setValue(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                    // nim tf2::Matrix3x3 m(q);
                    // nim double roll, pitch, yaw;
                    // nim m.getRPY(roll, pitch, yaw);
                    
                    
                    // nim const double heading = (M_PI * 0.5 - yaw) * 180.0 / M_PI;
                            const double heading = msg.participant_data.motion_state.yaw_angle;

                    auto sumopos = libsumo::Simulation::convertGeo(msg.participant_data.motion_state.x, msg.participant_data.motion_state.y, true);
                    // following line: keepRoute=2. see
                    // https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#move_to_xy_0xb4 for details
                    moveToXY(sumoid, "", 0, sumopos.x, sumopos.y, heading, 2);
                    setSpeed(sumoid, msg.participant_data.motion_state.vx);
                }
            }

         
            void init_sumo()
            {
                std::cout << "SUMO INIT"<< std::endl;
                int port = -1;
                step_length = 0.05;
                std::string cfg_file;
                declare_parameter( "sumo config file", "" );
                get_parameter( "sumo config file", cfg_file);
                declare_parameter( "sumo step length", 0.01 );
                get_parameter("sumo step length", step_length);
                if (cfg_file.empty())
                {
                    throw std::runtime_error("Error: No config file for sumo provided.");
                }
                // cfg_file_ = "/home/fascar/catkin_ws/src/adore/adore_if_ros_demos/demo005.sumocfg";
                std::vector<std::string> sumoargs;
                sumoargs.push_back("sumo");
                sumoargs.push_back("-c");
                sumoargs.push_back(cfg_file);
                sumoargs.push_back("--step-length");
                sumoargs.push_back(std::to_string(step_length));
                if (port > -1)
                {
                    sumoargs.push_back("--remote-port");
                    sumoargs.push_back(std::to_string(port));
                }

                while (true)
                {
                    try
                    {
                        std::cout << "load sumo ..." << std::flush;
                       // libsumo::Simulation::load(sumoargs);
                        libsumo::Simulation::start(sumoargs);
                        std::cout << " done." << std::endl;
                        break;
                    }
                    catch (const std::exception& exc)
                    {
                        std::cout << exc.what() << std::endl;
                        std::cout << "Arguments for sumo:" << std::endl;
                        for (auto a : sumoargs)
                        {
                            std::cout << a << std::endl;
                        }
                        std::cout << "try again ..." << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                std::cout << "started sumo with config file " << cfg_file << " and step length " << step_length
                          << std::endl;
                tSUMO0 = libsumo::Simulation::getTime();
                tSUMO = tSUMO0;
                tROS0 = this->get_clock()->now().nanoseconds();
                ros_time = tROS0;
                publisher_ = this->create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>("/ego_vehicle/traffic_participants",5);
                                                                                               // nim "traffic/"
                                                                                               // nim "agg",
                                                                                               // nim 5);
            // nim    get_parameter("PARAMS/V2X_TL/UTMZone", sumotls2ros.utm_zone_);
            // nim    get_parameter("PARAMS/V2X_TL/SouthHemi", sumotls2ros.is_south_hemi_);
            // nim    get_parameter("PARAMS/V2X_TL/UseSystemTime", sumotls2ros._use_system_time);
            // nim    get_parameter("PARAMS/V2X_TL/EnableSPATTiming", sumotls2ros._generate_spat_timing);
            // nim    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"SPAT/MAP Generation Parameters");
            // nim    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"-- UTM-Zone: " << sumotls2ros.utm_zone_);
            // nim    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"-- South. Hemi.: " << sumotls2ros.is_south_hemi_);
            // nim    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"-- use system time: " << sumotls2ros._use_system_time);
            // nim    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"-- enable spat timing: " << sumotls2ros._generate_spat_timing);
            // nim    RCLCPP_INFO_STREAM_ONCE(this->get_logger(),"-------------------------------");
           // nim     mapem_publisher_ =
           // nim         this->create_publisher<dsrc_v2_mapem_pdu_descriptions_msgs::msg::MAPEM>("/SIM/v2x/MAPEM", 100);
            // nim    spatem_publisher_ =
            // nim        this->create_publisher<dsrc_v2_spatem_pdu_descriptions_msgs::msg::SPATEM>("/SIM/v2x/SPATEM",100);
                //subscriber1_ = this->create_subscription<std_msgs::msg::Float64>("/SIM/utcccc", 1, std::bind( &Timer::receive, &timer_
                //                                                                            , std::placeholders::_1 ) );
                subscriber2_ = this->create_subscription<adore_ros2_msgs::msg::TrafficParticipantDetection>(
                    "/SIM/traffic", 100, std::bind(&ROSVehicleSet::receive, &rosVehicleSet_, std::placeholders::_1));
                sumo_rosveh_prefix_ = "rosvehicle";
                sumo_rosped_prefix_ = "rospedestrian";
                last_assigned_int_id_ = 1000;
            }

            void closeSumo()
            {
                libsumo::Simulation::close();
            }
        };
    }  // namespace sumo_if_ros
}  // namespace adore