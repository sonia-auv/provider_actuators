/**
 * \file	provider_actuators_node.h
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

#ifndef PROVIDER_ACTUATORS_PROVIDER_ACTUATOR_NODE_H_
#define PROVIDER_ACTUATORS_PROVIDER_ACTUATOR_NODE_H_

#include <ros/node_handle.h>
#include <sonia_common/SendRS485Msg.h>
#include <sonia_common/ActuatorDoAction.h>
#include <sonia_common/ActuatorDoActionSrv.h>

namespace provider_actuators {

class ProviderActuatorsNode {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ProviderActuatorsNode(const ros::NodeHandlePtr &nh);
  ~ProviderActuatorsNode();

  void Spin();

private:

    const ros::NodeHandlePtr nh;

    ros::Publisher rs485_publisherRx;
    ros::Subscriber rs485_subscriberTx;

    ros::Subscriber doActionSubscriber;
    ros::ServiceServer doActionService;

    void CommunicationDataCallback(const sonia_common::SendRS485Msg::ConstPtr &receivedData);
    void HandleDroppersCallback(sonia_common::SendRS485Msg::_data_type data);
    void HandleTorpedosCallback(sonia_common::SendRS485Msg::_data_type data);
    void HandleArmCallback(sonia_common::SendRS485Msg::_data_type data);

    void DoActionCallback(const sonia_common::ActuatorDoAction::ConstPtr &receivedData);
    bool DoActionSrvCallback(sonia_common::ActuatorDoActionSrv::Request &request, sonia_common::ActuatorDoActionSrv::Response &response);

};

}  // namespace provider_actuators

#endif  // PROVIDER_ACTUATORS_PROVIDER_ACTUATOR_NODE_H_
