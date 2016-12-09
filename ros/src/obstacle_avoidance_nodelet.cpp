/*
 * Copyright (c) 2016 Parrot S.A.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT COMPANY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "obstacle_avoidance_nodelet.hpp"

#include <array>
#include <cv_bridge/cv_bridge.h>

void ObstacleAvoidanceNodelet::onInit()
{
    ROS_DEBUG("Obstacle avoidance nodelet initialization");

    m_nh = getNodeHandle();

    m_depthSub =
        m_nh.subscribe("/depth_map/image", 1,
                       &ObstacleAvoidanceNodelet::onNewDepthMapImage, this);

    m_attitudeChangedSub =
        m_nh.subscribe("/bebop_node/attitudeChanged", 1,
                       &ObstacleAvoidanceNodelet::attitudeChangedCb, this);

    m_speedChangedSub =
        m_nh.subscribe("/bebop_node/speedChanged", 1,
                       &ObstacleAvoidanceNodelet::speedChangedCb, this);

    m_pilotingVelocitySub =
        m_nh.subscribe("/bebop_node/pilotingVelocity", 1,
                       &ObstacleAvoidanceNodelet::pilotingVelocityCb, this);

    m_avoidPilotingVelocityPub = m_nh.advertise<geometry_msgs::Twist>(
        "pilotingVelocityWithAvoidance", 1);
}

void ObstacleAvoidanceNodelet::onNewDepthMapImage(
    const sensor_msgs::ImageConstPtr &imageMsg)
{
    ROS_DEBUG("New depth map");
    cv_bridge::CvImageConstPtr img_bridge;
    img_bridge = cv_bridge::toCvShare(imageMsg,
                                      sensor_msgs::image_encodings::TYPE_32FC1);

    std::array<float, 3> droneHorizSpeed;

    droneHorizSpeed[0] = std::cos(m_attitude[2]) * m_speed[0] +
                         std::sin(m_attitude[2]) * m_speed[1];
    droneHorizSpeed[1] = -std::sin(m_attitude[2]) * m_speed[0] +
                         std::cos(m_attitude[2]) * m_speed[1];
    droneHorizSpeed[2] = m_speed[2];

    m_avoid.computeSpeed(droneHorizSpeed[0], -droneHorizSpeed[1],
                         -droneHorizSpeed[2], m_attitude[0], m_attitude[1],
                         m_attitude[2], img_bridge->image);
}

void ObstacleAvoidanceNodelet::pilotingVelocityCb(
    const geometry_msgs::TwistConstPtr &msg)
{
    if (msg->linear.x > 0)
    {
        std::array<float, 3> newPilotingCommand =
            m_avoid.updateAvoidCMD(msg->linear.y, msg->linear.x, msg->linear.z);

        if (m_meanAvoidIterator == 0)
        {
            m_droneCMD =
                Eigen::Vector3d(newPilotingCommand[0], newPilotingCommand[1],
                                newPilotingCommand[2]);
            m_meanAvoidIterator++;
        }
        else
        {
            m_droneCMD +=
                Eigen::Vector3d(newPilotingCommand[0], newPilotingCommand[1],
                                newPilotingCommand[2]);
            m_droneCMD /= 2.;

            geometry_msgs::Twist pilotingCommandMsg;
            pilotingCommandMsg.linear.x = m_droneCMD[1];
            pilotingCommandMsg.linear.y = m_droneCMD[0];
            pilotingCommandMsg.linear.z = m_droneCMD[2];
            pilotingCommandMsg.angular.z = msg->angular.z;
            ROS_DEBUG("Publishing avoid command: %f %f %f", m_droneCMD[1],
                      m_droneCMD[0], m_droneCMD[2]);
            m_avoidPilotingVelocityPub.publish(pilotingCommandMsg);
            m_meanAvoidIterator = 0;
        }
    }
    else
    {
        m_avoidPilotingVelocityPub.publish(msg);
    }
}

void ObstacleAvoidanceNodelet::attitudeChangedCb(
    const geometry_msgs::Vector3 &msg)
{
    m_attitude = {{static_cast<float>(msg.x), static_cast<float>(msg.y),
                   static_cast<float>(msg.z)}};
}

void ObstacleAvoidanceNodelet::speedChangedCb(const geometry_msgs::Vector3 &msg)
{
    m_speed = {{static_cast<float>(msg.x), static_cast<float>(msg.y),
                static_cast<float>(msg.z)}};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ObstacleAvoidanceNodelet, nodelet::Nodelet);
