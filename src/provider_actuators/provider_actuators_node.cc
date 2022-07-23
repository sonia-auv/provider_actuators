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

        rs485_publisherRx = nh->advertise<sonia_common::SendRS485Msg>("/interface_rs485/dataRx", 100);
        rs485_subscriberTx = nh->subscribe("/interface_rs485/dataTx", 100, &ProviderActuatorsNode::CommunicationDataCallback, this);

        doActionSubscriber = nh->subscribe("/provider_actuators/do_action_to_actuators", 100, &ProviderActuatorsNode::DoActionCallback, this);
        doActionPublisher = nh->advertise<sonia_common::ActuatorSendReply>("/provider_actuators/do_action_from_actuators", 100);

        isAliveService = nh->advertiseService("provider_actuators/isAlive", &ProviderActuatorsNode::IsAlive, this);
    }

    //------------------------------------------------------------------------------
    //
    ProviderActuatorsNode::~ProviderActuatorsNode() {
        rs485_subscriberTx.shutdown();
        doActionSubscriber.shutdown();
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
            for(std::vector<storedInfo>::iterator i = timeouts.begin(); i != timeouts.end();){
                if(i->timeout > 1){
                    i->timeout -= 1;
                    i++;
                }
                else{
                    sonia_common::ActuatorSendReply reply;
                    reply.element = i->element;
                    reply.side = i->side;
                    reply.response = sonia_common::ActuatorSendReply::RESPONSE_TIMED_OUT;
                    doActionPublisher.publish(reply);
                    i = timeouts.erase(i);
                }
            }
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
        if (data[0] == sonia_common::SendRS485Msg::DATA_IO_DROPPER_PORT){
            ROS_INFO("Dropper port activated");
            SendActionPublisherSuccess(sonia_common::ActuatorSendReply::ELEMENT_DROPPER, sonia_common::ActuatorSendReply::SIDE_PORT);
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_DROPPER_STARBOARD){
            ROS_INFO("Dropper starboard activated");
            SendActionPublisherSuccess(sonia_common::ActuatorSendReply::ELEMENT_DROPPER, sonia_common::ActuatorSendReply::SIDE_STARBOARD);
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_COULD_NOT_COMPLETE){
            ROS_INFO("Dropper was not activated");
            SendActionPublisherFailure(sonia_common::ActuatorSendReply::ELEMENT_DROPPER);
        }
    }

    void ProviderActuatorsNode::HandleTorpedosCallback(sonia_common::SendRS485Msg::_data_type data) {
        if (data[0] == sonia_common::SendRS485Msg::DATA_IO_TORPEDO_PORT){
            ROS_INFO("Torpedo port activated");
            SendActionPublisherSuccess(sonia_common::ActuatorSendReply::ELEMENT_TORPEDO, sonia_common::ActuatorSendReply::SIDE_PORT);
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_TORPEDO_STARBOARD){
            ROS_INFO("Torpedo starboard activated");
            SendActionPublisherSuccess(sonia_common::ActuatorSendReply::ELEMENT_TORPEDO, sonia_common::ActuatorSendReply::SIDE_STARBOARD);
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_COULD_NOT_COMPLETE){
            ROS_INFO("Torpedo was not activated");
            SendActionPublisherFailure(sonia_common::ActuatorSendReply::ELEMENT_TORPEDO);
        }
    }

    void ProviderActuatorsNode::HandleArmCallback(sonia_common::SendRS485Msg::_data_type data) {
        if (data[0] == sonia_common::SendRS485Msg::DATA_IO_ARM_OPEN){
            ROS_INFO("Arm Opened");
            SendActionPublisherSuccess(sonia_common::ActuatorSendReply::ELEMENT_ARM, sonia_common::ActuatorSendReply::ARM_OPEN);
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_ARM_CLOSE){
            ROS_INFO("Arm Closed");
            SendActionPublisherSuccess(sonia_common::ActuatorSendReply::ELEMENT_ARM, sonia_common::ActuatorSendReply::ARM_CLOSE);
        }
        else if (data[0] == sonia_common::SendRS485Msg::DATA_IO_COULD_NOT_COMPLETE){
            ROS_INFO("Arm was not activated");
            SendActionPublisherFailure(sonia_common::ActuatorSendReply::ELEMENT_ARM);
        }
    }

    void ProviderActuatorsNode::SendActionPublisherSuccess(uint8_t element, uint8_t side){
        for(std::vector<storedInfo>::iterator i = timeouts.begin(); i != timeouts.end(); ){
            if(i->element == element && i->side == side){
                sonia_common::ActuatorSendReply reply;
                reply.element = element;
                reply.side = side;
                reply.response = sonia_common::ActuatorSendReply::RESPONSE_SUCCESS;
                doActionPublisher.publish(reply);
                i = timeouts.erase(i);
            } else {
                i++;
            }
        }
    }

    void ProviderActuatorsNode::SendActionPublisherFailure(uint8_t element){
        for(std::vector<storedInfo>::iterator i = timeouts.begin(); i != timeouts.end(); ){
            if(i->element == element){
                sonia_common::ActuatorSendReply reply;
                reply.element = element;
                reply.side = i->side;
                reply.response = sonia_common::ActuatorSendReply::RESPONSE_FAILURE;
                doActionPublisher.publish(reply);
                i = timeouts.erase(i);
            } else {
                i++;
            }
        }
    }

    void ProviderActuatorsNode::DoActionCallback(const sonia_common::ActuatorDoAction::ConstPtr &receivedData) {
        
        for(std::vector<storedInfo>::iterator i = timeouts.begin(); i != timeouts.end(); i++){
            if(i->element == receivedData->element && i->side == receivedData->side){
                return;
            }
        }

        storedInfo temp;
        temp.element = receivedData->element;
        temp.side = receivedData->side;
        temp.timeout = 5; 
        timeouts.push_back(temp);

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

    // empty service in order to detect if the node is still alive by proc_fault.
    bool ProviderActuatorsNode::IsAlive(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {return true;}
}
