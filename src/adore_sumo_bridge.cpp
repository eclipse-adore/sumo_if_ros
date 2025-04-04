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


int main(int argc, char** argv)
{
    rclcpp::init( argc, argv );
    rclcpp::spin(std::make_shared<adore::sumo_if_ros::SUMOTrafficToROS>());
    rclcpp::shutdown();
    return 0;
}