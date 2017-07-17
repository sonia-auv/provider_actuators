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

        rs485_publisherRx =
                nh->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataRx", 100);

        rs485_subscriberTx =
                nh->subscribe("/interface_rs485/dataTx", 100, &ProviderActuatorsNode::CommunicationDataCallback, this);

        doActionSubscriber = nh->subscribe("/provider_actuators/do_action", 100, &ProviderActuatorsNode::DoActionCallback, this);

        doActionService = nh->advertiseService("/provider_actuators/do_action_srv", &ProviderActuatorsNode::DoActionSrvCallback, this);

    }

    //------------------------------------------------------------------------------
    //
    ProviderActuatorsNode::~ProviderActuatorsNode() {
        rs485_subscriberTx.shutdown();
    }

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

    void ProviderActuatorsNode::CommunicationDataCallback(const interface_rs485::SendRS485Msg::ConstPtr &receivedData) {


        if (receivedData->slave == interface_rs485::SendRS485Msg::SLAVE_IO_CTR)
        {

            switch (receivedData->cmd)
            {

                case interface_rs485::SendRS485Msg::CMD_IO_TEMP:
                    HandleTempCallback(receivedData->data);
                    break;

                case interface_rs485::SendRS485Msg::CMD_IO_DROPPER_PORT:
                case interface_rs485::SendRS485Msg::CMD_IO_DROPPER_STARBOARD:
                    HandleDroppersCallback(receivedData->cmd, receivedData->data);
                    break;

                case interface_rs485::SendRS485Msg::CMD_IO_TORPEDO_PORT:
                case interface_rs485::SendRS485Msg::CMD_IO_TORPEDO_STARBOARD:
                    HandleTorpedosCallback(receivedData->cmd, receivedData->data);
                    break;

                case interface_rs485::SendRS485Msg::CMD_IO_LEAK_SENSOR_BACK:
                case interface_rs485::SendRS485Msg::CMD_IO_LEAK_SENSOR_FRONT:
                case interface_rs485::SendRS485Msg::CMD_IO_LEAK_SENSOR_RIGHT:
                case interface_rs485::SendRS485Msg::CMD_IO_LEAK_SENSOR_LEFT:
                    HandleLeakSensorsCallback(receivedData->cmd);
                    break;

            }

        }

    }

    void ProviderActuatorsNode::HandleTempCallback(interface_rs485::SendRS485Msg::_data_type data) {

        union Temperature
        {
            unsigned char bytes[4];
            float temperature;
        };

        Temperature temperatureTransfert;

        temperatureTransfert.bytes[0] = data[0];
        temperatureTransfert.bytes[1] = data[1];
        temperatureTransfert.bytes[2] = data[2];
        temperatureTransfert.bytes[3] = data[3];

        float temperature = temperatureTransfert.temperature;

        // TODO Send msg

        ROS_INFO("Board IO temperature : %f", temperature);

    }

    void ProviderActuatorsNode::HandleDroppersCallback(interface_rs485::SendRS485Msg::_cmd_type cmd, interface_rs485::SendRS485Msg::_data_type data) {

        std::string side;

        if (cmd == interface_rs485::SendRS485Msg::CMD_IO_DROPPER_PORT)
        {
            side = "port";
        }
        else if (cmd == interface_rs485::SendRS485Msg::CMD_IO_DROPPER_STARBOARD)
        {
            side = "starboard";
        }

        ROS_INFO("Dropper %s : %d", side.data(), data[0]);
        // TODO send msg

    }

    void ProviderActuatorsNode::HandleTorpedosCallback(interface_rs485::SendRS485Msg::_cmd_type cmd, interface_rs485::SendRS485Msg::_data_type data) {

        std::string side;

        if (cmd == interface_rs485::SendRS485Msg::CMD_IO_TORPEDO_PORT)
        {
            side = "port";
        }
        else if (cmd == interface_rs485::SendRS485Msg::CMD_IO_TORPEDO_STARBOARD)
        {
            side = "starboard";
        }

        ROS_INFO("Torpedo %s : %d", side.data(), data[0]);
        // TODO send msg
    }

    void ProviderActuatorsNode::HandleLeakSensorsCallback(interface_rs485::SendRS485Msg::_cmd_type cmd) {

        std::string side;

        if (cmd == interface_rs485::SendRS485Msg::CMD_IO_LEAK_SENSOR_FRONT)
        {
            side = "front";
        }
        else if (cmd == interface_rs485::SendRS485Msg::CMD_IO_LEAK_SENSOR_LEFT)
        {
            side = "left";
        }
        else if (cmd == interface_rs485::SendRS485Msg::CMD_IO_LEAK_SENSOR_RIGHT)
        {
            side = "right";
        }
        else if (cmd == interface_rs485::SendRS485Msg::CMD_IO_LEAK_SENSOR_BACK)
        {
            side = "back";
        }

        ROS_INFO("Leak on %s", side.data());

        // TODO Send msg

    }

    void ProviderActuatorsNode::DoActionCallback(const DoAction::ConstPtr &receivedData) {

        interface_rs485::SendRS485Msg rs485Msg;

        rs485Msg.slave = interface_rs485::SendRS485Msg::SLAVE_IO_CTR;

        if (receivedData->element == DoAction::ELEMENT_DROPPER
            && receivedData->side == DoAction::SIDE_PORT)
        {
            rs485Msg.cmd = interface_rs485::SendRS485Msg::CMD_IO_DROPPER_PORT;
        }
        else if (receivedData->element == DoAction::ELEMENT_DROPPER
                 && receivedData->side == DoAction::SIDE_STARBOARD)
        {
            rs485Msg.cmd = interface_rs485::SendRS485Msg::CMD_IO_DROPPER_STARBOARD;
        }
        else if (receivedData->element == DoAction::ELEMENT_TORPEDO
                 && receivedData->side == DoAction::SIDE_PORT)
        {
            rs485Msg.cmd = interface_rs485::SendRS485Msg::CMD_IO_TORPEDO_PORT;
        }
        else if (receivedData->element == DoAction::ELEMENT_TORPEDO
                 && receivedData->side == DoAction::SIDE_STARBOARD)
        {
            rs485Msg.cmd = interface_rs485::SendRS485Msg::CMD_IO_TORPEDO_STARBOARD;
        }

        rs485Msg.data.push_back(receivedData->action);

        rs485_publisherRx.publish(rs485Msg);

    }

    bool ProviderActuatorsNode::DoActionSrvCallback(DoActionSrv::Request &request,
                                                    DoActionSrv::Response &response) {

        DoAction::ConstPtr msg;
        msg->action = request.action;
        msg->element = request.element;
        msg->side = request.side;

        DoActionCallback(msg);

        return true;
    }

}  // namespace provider_actuators
