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
                nh->advertise<sonia_common::SendRS485Msg>("/interface_rs485/dataRx", 100);

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
    void ProviderActuatorsNode::Spin()
    {
        ros::Rate r(1);  // 1 hz

        while (ros::ok()) 
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProviderActuatorsNode::CommunicationDataCallback(const sonia_common::SendRS485Msg::ConstPtr &receivedData) 
    {
        if (receivedData->slave == sonia_common::SendRS485Msg::SLAVE_IO)
        {
            switch (receivedData->cmd)
            {
                case sonia_common::SendRS485Msg::CMD_IO_DROPPER_ACTION:
                    HandleDroppersCallback(receivedData->cmd, receivedData->data);
                    break;

                case sonia_common::SendRS485Msg::CMD_IO_TORPEDO_ACTION:
                    HandleTorpedosCallback(receivedData->cmd, receivedData->data);
                    break;

                case sonia_common::SendRS485Msg::CMD_IO_ARM_ACTION:
                    HandleArmCallback(receivedData->cmd, receivedData->data);
                    break;
            }
        }
    }

    void ProviderActuatorsNode::HandleTempCallback(sonia_common::SendRS485Msg::_data_type data) {

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

    void ProviderActuatorsNode::HandleDroppersCallback(sonia_common::SendRS485Msg::_cmd_type cmd, sonia_common::SendRS485Msg::_data_type data) {

        std::string side;

        if (data[0] == sonia_common::SendRS485Msg::DATA_IO_DROPPER_PORT)
        {
            side = "port";
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_DROPPER_STARBOARD)
        {
            side = "starboard";
        }

        ROS_INFO("Dropper %s : %d", side.data(), data[0]);
        // TODO send msg

    }

    void ProviderActuatorsNode::HandleTorpedosCallback(sonia_common::SendRS485Msg::_cmd_type cmd, sonia_common::SendRS485Msg::_data_type data) {

        std::string side;

        if (data[0] == sonia_common::SendRS485Msg::DATA_IO_TORPEDO_PORT)
        {
            side = "port";
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_TORPEDO_STARBOARD)
        {
            side = "starboard";
        }

        ROS_INFO("Torpedo %s : %d", side.data(), data[0]);
        // TODO send msg
    }

    void ProviderActuatorsNode::HandleLeakSensorsCallback(sonia_common::SendRS485Msg::_cmd_type cmd) {

        std::string side;

        // if (cmd == sonia_common::SendRS485Msg::CMD_IO_LEAK_SENSOR_FRONT)
        // {
        //     side = "front";
        // }
        // else if (cmd == sonia_common::SendRS485Msg::CMD_IO_LEAK_SENSOR_LEFT)
        // {
        //     side = "left";
        // }
        // else if (cmd == sonia_common::SendRS485Msg::CMD_IO_LEAK_SENSOR_RIGHT)
        // {
        //     side = "right";
        // }
        // else if (cmd == sonia_common::SendRS485Msg::CMD_IO_LEAK_SENSOR_BACK)
        // {
        //     side = "back";
        // }

        ROS_INFO("Leak on %s", side.data());

        // TODO Send msg

    }

    void ProviderActuatorsNode::HandleArmCallback(sonia_common::SendRS485Msg::_cmd_type cmd, sonia_common::SendRS485Msg::_data_type data) {

        std::string side;

        if (cmd == sonia_common::SendRS485Msg::CMD_IO_ARM_OPEN)
        {
            side = "open";
        }
        else if (cmd == sonia_common::SendRS485Msg::CMD_IO_ARM_CLOSE)
        {
            side = "close";
        }

        ROS_INFO("ARM %s : %d", side.data(), data[0]);
        // TODO send msg
    }

    void ProviderActuatorsNode::DoActionCallback(const sonia_common::ActuatorDoAction::ConstPtr &receivedData) {

        sonia_common::SendRS485Msg rs485Msg;

        rs485Msg.slave = sonia_common::SendRS485Msg::SLAVE_IO;

        if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_DROPPER
            && receivedData->side == sonia_common::ActuatorDoAction::SIDE_PORT)
        {
            rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_DROPPER_ACTION;
            rs485Msg.data.push_back(sonia_common::SendRS485Msg::DATA_IO_DROPPER_PORT); 
        }
        else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_DROPPER
                 && receivedData->side == sonia_common::ActuatorDoAction::SIDE_STARBOARD)
        {
            rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_DROPPER_ACTION;
            rs485Msg.data.push_back(sonia_common::SendRS485Msg::DATA_IO_DROPPER_STARBOARD);
        }
        // else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_TORPEDO
        //          && receivedData->side == sonia_common::ActuatorDoAction::SIDE_PORT)
        // {
        //     rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_TORPEDO_PORT;
        // }
        // else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_TORPEDO
        //          && receivedData->side == sonia_common::ActuatorDoAction::SIDE_STARBOARD)
        // {
        //     rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_TORPEDO_STARBOARD;
        // }

        // else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_ARM
        //          && receivedData->side == sonia_common::ActuatorDoAction::ARM_OPEN)
        // {
        //     rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_ARM_OPEN;
        // }

        // else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_ARM
        //          && receivedData->side == sonia_common::ActuatorDoAction::ARM_CLOSE)
        // {
        //     rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_ARM_CLOSE;
        // }

        // rs485Msg.data.push_back(receivedData->action);

        rs485_publisherRx.publish(rs485Msg);

    }

    bool ProviderActuatorsNode::DoActionSrvCallback(sonia_common::ActuatorDoActionSrv::Request &request,
                                                    sonia_common::ActuatorDoActionSrv::Response &response) {

        sonia_common::ActuatorDoAction::Ptr msg(new sonia_common::ActuatorDoAction());
        msg->action = request.action;
        msg->element = request.element;
        msg->side = request.side;

        DoActionCallback(msg);

        return true;
    }

}  // namespace provider_actuators
