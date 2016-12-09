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

#ifndef OBSTACLE_AVOIDANCE_NODELET_HPP
#define OBSTACLE_AVOIDANCE_NODELET_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "episkopi/Avoid.hpp"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Image.h"

class ObstacleAvoidanceNodelet : public nodelet::Nodelet
{
private:
    void onInit();
    void onNewDepthMapImage(const sensor_msgs::ImageConstPtr &imageMsg);
    void pilotingVelocityCb(const geometry_msgs::TwistConstPtr &msg);
    void attitudeChangedCb(const geometry_msgs::Vector3 &msg);
    void speedChangedCb(const geometry_msgs::Vector3 &msg);

private:
    episkopi::Avoid m_avoid;

    ros::NodeHandle m_nh;

    ros::Subscriber m_depthSub;
    ros::Subscriber m_pilotingVelocitySub;
    ros::Subscriber m_attitudeChangedSub;
    ros::Subscriber m_speedChangedSub;

    ros::Publisher m_avoidPilotingVelocityPub;

    std::array<float, 3> m_attitude;
    std::array<float, 3> m_speed;

    Eigen::Vector3d m_droneCMD = Eigen::Vector3d(0, 0, 0);
    int m_meanAvoidIterator = 0;
};

#endif // OBSTACLEAVOIDANCENODELET_HPP
