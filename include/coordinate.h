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
 ********************************************************************************/
#pragma once

namespace adore
{
    namespace sumo_if_ros
    {
        struct Coordinate
        {
            Coordinate():m_X(0.0),m_Y(0.0),m_Z(0.0){}
            Coordinate(double x,double y,double z):m_X(x),m_Y(y),m_Z(z){}
            double m_X,m_Y,m_Z;
            bool operator==(const Coordinate& other)const
            {
                static const double tol = 1e-2;
                const double dx = m_X-other.m_X;
                const double dy = m_Y-other.m_Y;
                const double dz = m_Z-other.m_Z;
                const double dist2 = dx*dx+dy*dy+dz*dz;
                return dist2<=tol*tol;
            }
        };
    }
}