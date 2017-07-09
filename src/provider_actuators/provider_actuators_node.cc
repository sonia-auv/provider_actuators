/**
 * \file	provider_actuators_node.cc
 * \author	Marc-Antoine Couture <coumarc9@outlook.com>
 * \date	07/08/2017
 *
 * \copyright Copyright (c) 2017 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "provider_actuators/provider_actuators_node.h"

namespace provider_actuators {

    //==============================================================================
    // C / D T O R S   S E C T I O N
    //------------------------------------------------------------------------------
    //
    ProviderActuatorsNode::ProviderActuatorsNode(const ros::NodeHandlePtr &nh)
        : nh(nh)
    {
    }

    //------------------------------------------------------------------------------
    //
    ProviderActuatorsNode::~ProviderActuatorsNode() {}

    //==============================================================================
    // M E T H O D   S E C T I O N
    //------------------------------------------------------------------------------
    //
    void ProviderActuatorsNode::Spin() {

      ros::Rate r(1);  // 1 hz
      while (ros::ok()) {
        ros::spinOnce();

       r.sleep();
      }
    }


}  // namespace provider_actuators
