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

#ifndef EPISKOPI_AVOID_HPP
#define EPISKOPI_AVOID_HPP

#include "AvoidSettings.hpp"

#include <Eigen/Geometry>
#include <array>
#include <opencv2/core/core.hpp>

namespace episkopi
{

class Avoid
{

public:
    /// update the incomming commands according to the algorithm
    std::array<float, 3>
    updateAvoidCMD(float rollCMD, float pitchCMD, float gazCMD);

    /// get minimum time before drone collides with something
    float getMinImpactTime() const;

    /// get avoidance criterion
    bool isAvoiding() const;

    /// compute speed
    void computeSpeed(float v1,
                      float v2,
                      float v3,
                      float rotZ,
                      float rotY,
                      float rotX,
                      const cv::Mat1f &image);

private:
    /// computation of negative forces
    void repulsiveForce(const cv::Mat1f &image);

    /// computation of needed drone speed
    void droneSpeed(float normVIn);

    /// computation of time before impact of one voxel(x, y, z)
    float impactTime(Eigen::Vector3f voxelCoordinates);

    /// Notify User
    /// If there is no obstacle, all leds are green
    /// If the drone is avoiding to the right, the leds on the left side are red
    /// If the drone is avoiding to the left, the leds on the right side are red
    /// If the drone face a wall and won't move forward, all leds are red
    void userNotification() const;

private:
    /// algorithm settings
    AvoidSettings m_settings;

    /// the drone coordinate system is set as follow :
    /// x axis is upward
    /// y axis is rightward
    /// z axis is toward

    Eigen::Vector3f m_futurePos = Eigen::Vector3f(0., 0., 0.);

    /// Output speed of the drone, in body referential
    Eigen::Vector3f m_speedOut = Eigen::Vector3f(0., 0., 0.);

    /// Input speed of the drone, in body referential
    Eigen::Vector3f m_speedIn = Eigen::Vector3f(0., 0., 0.);

    /// Rotation of the drone on the Z (roll), Y (pitch)
    /// and X (yaw) axis in [°]
    Eigen::Vector3f m_angles = Eigen::Vector3f(0., 0., 0.);

    /// repulsive force computed from the voxels which have
    /// an impact time < m_settings.impactTimeThreshold
    Eigen::Vector3f m_forceVoxelWithImpact = Eigen::Vector3f (0., 0., 0.);

    /// repulsive force computed from the voxels which have
    /// an impact time > m_settings.impactTimeThreshold
    Eigen::Vector3f m_forceVoxelWithoutImpact = Eigen::Vector3f (0., 0., 0.);

    /// Attractive forces used for the FlightPlan mode
    Eigen::Vector3f m_attractiveForce = Eigen::Vector3f (0., 0., 0.);

    float m_minImpactTime = 0;

    bool m_stopWall = false; ///< wall detection parameter
    std::array<bool, 3> m_avoidingXYZ = {{false, false, false}};
};

} // namespace episkopi

#endif // EPISKOPI_AVOID_HPP
