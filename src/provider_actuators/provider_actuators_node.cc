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

#include <unistd.h>
#include "provider_actuators/provider_actuators_node.h"

bool droppersActivated = false;
bool torpedoesActivated = false;
bool armActivated = false;

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
                    HandleDroppersCallback(receivedData->data);
                    break;

                case sonia_common::SendRS485Msg::CMD_IO_TORPEDO_ACTION:
                    HandleTorpedosCallback(receivedData->data);
                    break;

                case sonia_common::SendRS485Msg::CMD_IO_ARM_ACTION:
                    HandleArmCallback(receivedData->data);
                    break;
            }
        }
    }

    void ProviderActuatorsNode::HandleDroppersCallback(sonia_common::SendRS485Msg::_data_type data) {
        ROS_ERROR("test1");
        std::string side;

        if (data[0] == sonia_common::SendRS485Msg::DATA_IO_DROPPER_PORT)
        {
            side = "port";
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_DROPPER_STARBOARD)
        {
            side = "starboard";
        }
        ROS_ERROR("test2");
        ROS_ERROR("%s", droppersActivated?"in function true":"in function false");
        ROS_ERROR("Dropper %s activated", side.data());
        ROS_ERROR("test3");
        droppersActivated = true;
        ROS_ERROR("%s", droppersActivated?"in function true":"in function false");
    }

    void ProviderActuatorsNode::HandleTorpedosCallback(sonia_common::SendRS485Msg::_data_type data) {

        std::string side;

        if (data[0] == sonia_common::SendRS485Msg::DATA_IO_TORPEDO_PORT)
        {
            side = "port";
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_TORPEDO_STARBOARD)
        {
            side = "starboard";
        }

        ROS_INFO("Torpedo %s activated", side.data());
        
        torpedoesActivated = true;
    }

    void ProviderActuatorsNode::HandleArmCallback(sonia_common::SendRS485Msg::_data_type data) {

        std::string side;

        if (data[0] == sonia_common::SendRS485Msg::DATA_IO_ARM_OPEN)
        {
            side = "open";
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_ARM_CLOSE)
        {
            side = "close";
        }

        ROS_INFO("ARM %s", side.data());
        
        armActivated = true;
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
        else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_TORPEDO
                 && receivedData->side == sonia_common::ActuatorDoAction::SIDE_PORT)
        {
            rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_TORPEDO_ACTION;
            rs485Msg.data.push_back(sonia_common::SendRS485Msg::DATA_IO_TORPEDO_PORT);
        }
        else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_TORPEDO
                 && receivedData->side == sonia_common::ActuatorDoAction::SIDE_STARBOARD)
        {
            rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_TORPEDO_ACTION;
            rs485Msg.data.push_back(sonia_common::SendRS485Msg::DATA_IO_TORPEDO_STARBOARD);
        }
        else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_ARM
                 && receivedData->side == sonia_common::ActuatorDoAction::ARM_OPEN)
        {
            rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_ARM_ACTION;
            rs485Msg.data.push_back(sonia_common::SendRS485Msg::DATA_IO_ARM_OPEN);
        }
        else if (receivedData->element == sonia_common::ActuatorDoAction::ELEMENT_ARM
                 && receivedData->side == sonia_common::ActuatorDoAction::ARM_CLOSE)
        {
            rs485Msg.cmd = sonia_common::SendRS485Msg::CMD_IO_ARM_ACTION;
            rs485Msg.data.push_back(sonia_common::SendRS485Msg::DATA_IO_ARM_CLOSE);
        }

        rs485_publisherRx.publish(rs485Msg);
    }

    bool ProviderActuatorsNode::DoActionSrvCallback(sonia_common::ActuatorDoActionSrv::Request &request,
                                                    sonia_common::ActuatorDoActionSrv::Response &response) {
        
        torpedoesActivated = false;
        droppersActivated = false;
        armActivated = false;

        sonia_common::ActuatorDoAction::Ptr msg(new sonia_common::ActuatorDoAction());
        msg->action = request.action;
        msg->element = request.element;
        msg->side = request.side;

        DoActionCallback(msg);

        float timeout = 5; //Timeout value in seconds, can be edited to fit needs
        switch (request.element){
            case sonia_common::ActuatorDoAction::ELEMENT_DROPPER:
                while (!droppersActivated){
                    ROS_ERROR("%f", timeout);
                    if (timeout > 0){
                        ros::Duration(0.1).sleep();
                        ROS_ERROR("%s", droppersActivated?"out of function true":"out of function false");
                        timeout -= 0.1;
                    }
                    else{
                        ROS_ERROR("TIMED OUT!!!");
                        response.success = false;
                        return true;
                    }
                }
                ROS_ERROR("WOKRING!!!");
                response.success = true;
                return true;
            case sonia_common::ActuatorDoAction::ELEMENT_TORPEDO:
                while (!torpedoesActivated){
                    if (timeout > 0){
                        sleep(1);
                        timeout -= 1;
                    }
                    else{
                        response.success = false;
                        return true;
                    }
                }
                response.success = true;
                return true;
            case sonia_common::ActuatorDoAction::ELEMENT_ARM:
                while (!armActivated){
                    if (timeout > 0){
                        sleep(1);
                        timeout -= 1;
                    }
                    else{
                        response.success = false;
                        return true;
                    }
                }
                response.success = true;
                return true;
            default:
                return false;
        }
    }

}  // namespace provider_actuators
