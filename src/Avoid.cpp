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

#include "episkopi/Avoid.hpp"

#include "episkopi/AvoidSettings.hpp"

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sys/time.h>

namespace
{

template <typename T>
inline T simplePow(T base, unsigned exp)
{
    T result = base;
    for (unsigned i = 1; i < exp; ++i)
    {
        result *= base;
    }
    return result;
}

template <typename T>
T clamp(const T &x, const T &lower, const T &upper)
{
    return std::max(lower, std::min(x, upper));
}

} // unnamed namespace

namespace episkopi
{

void Avoid::repulsiveForce(const cv::Mat1f &img)
{
    // Distance between drone and obstacle
    float r = 0.f;

    // Drone future position if it keeps the same speed
    m_futurePos = Eigen::Vector3f(0.f, 0.f, 0.f);

    // Voxel coordinate
    Eigen::Vector3f voxelCoord = Eigen::Vector3f(0.f, 0.f, 0.f);
    Eigen::Vector3f voxelForce = Eigen::Vector3f(0.f, 0.f, 0.f);

    // Number of point closer than <wallDetectionDistance> to the drone
    int nbClosePoint = 0;

    // Intermediary sum for the repulsive force for each axis
    Eigen::Vector3f sumForceVoxelWithImpact = Eigen::Vector3f(0.f, 0.f, 0.f);
    Eigen::Vector3f sumForceVoxelWithoutImpact = Eigen::Vector3f(0.f, 0.f, 0.f);

    m_minImpactTime = std::numeric_limits<float>::max();

    // Computation of the sums on each axis
    for (unsigned short j = 0; j < img.rows; j++)
    {
        for (unsigned short i = 0; i < img.cols; i++)
        {
            // depth in m
            voxelCoord[2] = img.at<float>(j, i);
            if (std::isnan(voxelCoord[2]))
            {
                voxelCoord[2] = 0.;
            }

            // We want to ignore 0 because it means no informations
            // In stereo mode value under 0.5 m are wrong so we don't use them
            if (voxelCoord[2] <= m_settings.minDistToCompute)
            {
                continue;
            }
            // determination of voxel coordinates in the drone
            // coordinate system in m
            // vertical field of view = PI / 2.45
            voxelCoord[0] =
                -voxelCoord[2] *
                std::tan((j - img.rows / 2) * m_settings.vFov / img.rows);
            // horizontal field of view = PI / 2
            voxelCoord[1] =
                voxelCoord[2] *
                std::tan((i - img.cols / 2) * m_settings.hFov / img.cols);
            r = voxelCoord.norm();

            // counting number of points closer than 75 cm for wall
            // detection
            if (voxelCoord[2] < m_settings.wallDetectionDistance)
                nbClosePoint++;

            for (int i = 0; i <= 2; i++)
            {
                voxelForce[i] =
                    (voxelCoord[i] - m_futurePos[i]) /
                    simplePow(
                        r,
                        static_cast<float>(m_settings.repulsiveForceFactor[i]) +
                            1);
            }

            // if time before collision with voxel is under 3s it
            // contributes to the repulsive force
            float voxelImpactTime = impactTime(voxelCoord);
            if (voxelImpactTime < m_settings.impactTimeThreshold)
            {
                if (voxelImpactTime < m_minImpactTime)
                {
                    m_minImpactTime = voxelImpactTime;
                }
                sumForceVoxelWithImpact -= voxelForce;
            }
            else
            {
                sumForceVoxelWithoutImpact -= voxelForce;
            }
        }
    }

    // check if in front of a wall when the depthMap is composed of more than
    // wallDetectionMinSurfacePercent percent of close points
    if (m_stopWall == false &&
        nbClosePoint >
            img.rows * img.cols * m_settings.wallDetectionMinSurfacePercent)
    {
        m_stopWall = true;
        m_avoidingXYZ[2] = true;
        std::cout << "Stop there is a wall" << std::endl;
    }

    const Eigen::Vector3f dminPowForce = Eigen::Vector3f(
        simplePow(m_settings.minDistToObstacle,
                  static_cast<unsigned>(m_settings.repulsiveForceFactor[0])),
        simplePow(m_settings.minDistToObstacle,
                  static_cast<unsigned>(m_settings.repulsiveForceFactor[1])),
        simplePow(m_settings.minDistToObstacle,
                  static_cast<unsigned>(m_settings.repulsiveForceFactor[2])));

    assert(img.rows == 96 && img.cols == 96 &&
           "The size of the depth map must be 96x96");

    int balanceAttractiveForceFactor = 1;

    for (int i = 0; i <= 2; i++)
    {
        m_forceVoxelWithImpact[i] =
            std::max(std::abs(m_speedIn[i]), 1.f) * dminPowForce[i] *
            balanceAttractiveForceFactor * sumForceVoxelWithImpact[i] /
            m_settings.pixelStopThreshold[i];

        m_forceVoxelWithoutImpact[i] =
            std::max(std::abs(m_speedIn[i]), 1.f) * dminPowForce[i] *
            balanceAttractiveForceFactor * sumForceVoxelWithoutImpact[i] /
            m_settings.pixelStopThreshold[i];

        // If the repulsive force induce more than avoidPercentThreshold
        // of variation then we take it into account
        m_avoidingXYZ[i] =
            std::abs(sumForceVoxelWithImpact[i]) >
            std::max(std::abs(m_speedIn[i]) * m_settings.fRVinRatioThreshold,
                     m_settings.minimumAvoidSpeed);
    }
}

void Avoid::droneSpeed(const float normVIn)
{
    if (m_stopWall == true)
    {
        m_speedOut = Eigen::Vector3f(0.f, 0.f, 0.f);
        return;
    }
    else
    {
        m_speedOut = m_speedIn + m_forceVoxelWithImpact +
                     m_forceVoxelWithoutImpact + m_attractiveForce;
    }

    // check if -m_settings.vMax < m_vxOut,m_vyOut,m_vzOut < m_settings.vMax
    m_speedOut[0] = clamp(m_speedOut[0], -m_settings.vMax, m_settings.vMax);

    m_speedOut[1] = clamp(m_speedOut[1], -m_settings.vMax, m_settings.vMax);

    if (m_speedOut[1] > m_settings.vMax)
        m_speedOut[1] = m_settings.vMax;
    else
    {
        // drone should not go backwards but must be able to decelerate quickly
        // Consigne de vitesse négative mais pas de vitesse négative
        if (m_speedOut[2] > 0.2f)
        {
            if (m_speedOut[2] < -m_settings.vMax)
                m_speedOut[2] = -m_settings.vMax;
        }
        else if (m_speedOut[2] < 0)
            m_speedOut[2] = 0;
    }

    // drone should not go faster than original speed when avoiding
    float normV = m_speedOut.norm();
    if (normV > normVIn)
    {
        float coef = std::max(normVIn, 0.1f) /
                     std::max(normV, 0.1f);



        m_speedOut = m_speedOut * coef;
    }

    userNotification();
}

void Avoid::computeSpeed(float v1,
                         float v2,
                         float v3,
                         float roll,
                         float pitch,
                         float yaw,
                         const cv::Mat1f &image)
{
    // Repulsive forces should be enough but to avoid oscillations we put all
    // speeds to 0
    if (m_stopWall == true)
    {
        m_speedOut = Eigen::Vector3f(0.f, 0.f, 0.f);
    }
    else
    {
        m_avoidingXYZ = {{false, false, false}};
        m_speedIn = Eigen::Vector3f(v3, -v2, v1);
        m_angles = Eigen::Vector3f(roll, pitch, yaw);

        const float normVIn = m_speedIn.norm();

        repulsiveForce(image);

        droneSpeed(normVIn);
    }
}

void Avoid::userNotification() const
{
    if (isAvoiding())
    {
        if (m_speedOut[1] > m_speedIn[1])
        {
            std::cout << "Avoid left" << std::endl;
        }
        else if (m_speedOut[1] < m_speedIn[1])
        {
            std::cout << "Avoid right" << std::endl;
        }

        if (m_stopWall)
        {
            std::cout << "Avoid stop" << std::endl;
        }
    }
    else
    {
        std::cout << "Not avoiding" << std::endl;
    }
}

bool Avoid::isAvoiding() const
{
    return (m_avoidingXYZ[0] || m_avoidingXYZ[1] || m_avoidingXYZ[2]);
}

std::array<float, 3>
Avoid::updateAvoidCMD(float rollCMD, float pitchCMD, float gazCMD)
{
    // Do not change altitude during avoidance
    gazCMD = 0;
    std::array<float, 3> updatedCMD = {{rollCMD, pitchCMD, gazCMD}};

    if (!(m_avoidingXYZ[2] || m_avoidingXYZ[1]))
    {
        return updatedCMD;
    }
    else if (m_stopWall && m_speedIn[2] < 0.2f)
    {
        return {{0, 0, 0}};
    }

    if (m_avoidingXYZ[2] || m_avoidingXYZ[1])
    {
        if (m_speedIn[2] > 0.3f)
        {
            updatedCMD[1] = pitchCMD * 0.8f;

            if (m_stopWall && m_speedIn[2] > 0.2f)
            {
                updatedCMD[1] = -50;
            }
        }
        else
        {
            updatedCMD[1] = 20;
        }

        if (m_speedOut[1] >= m_speedIn[1])
        {
            if (m_speedIn[2] > 0.3f)
            {
                updatedCMD[0] = pitchCMD * 0.8f;
            }
            else
            {
                updatedCMD[0] = 20;
            }
        }
        else
        {
            if (m_speedIn[2] > 0.3f)
            {
                updatedCMD[0] = -pitchCMD * 0.8f;
            }
            else
            {
                updatedCMD[0] = -20;
            }
        }
    }

    return updatedCMD;
}

float Avoid::impactTime(Eigen::Vector3f voxelCoord)
{
    assert(voxelCoord[2] > 0.f && "The depth must be strictly positive");

    float impactTime = std::numeric_limits<float>::max();

    if (m_speedIn[2] <= 0)
    {
        return impactTime;
    }

    // then we check if the given point (x, y) is within a 1.6x1.6 window
    // Centered on (xFuture, yFuture)
    // if it's true then we will have an impact where the time before impact is
    // the time to reach this window
    if (std::abs(voxelCoord[0] - m_futurePos[0]) <
            m_settings.occupationWindow.height &&
        std::abs(voxelCoord[1] - m_futurePos[1]) <
            m_settings.occupationWindow.width)
    {
        impactTime = m_futurePos.norm() / m_speedIn.norm();
    }

    return impactTime;
}

} // namespace episkopi
