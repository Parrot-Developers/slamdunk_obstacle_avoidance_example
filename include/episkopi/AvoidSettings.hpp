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

#ifndef EPISKOPI_AVOIDSETTINGS_HPP
#define EPISKOPI_AVOIDSETTINGS_HPP

#include <array>

namespace episkopi
{

struct AvoidSettings
{

    // The minimum distance at which the point is taken into account in the
    // computation of the algorithm
    // FIXME: the minimum distance visible by the stereo algorithm is
    // approximately
    // 1.2 meters and according to Denis we should use points over 1.5 meters
    float minDistToCompute = 0.5f; ///< [m]
    // It is the closest distance to an obstacle the algorithm allow the drone
    // to be
    float minDistToObstacle = 0.7f; ///< [m]
    // The maximum speed at which the drone is allowed to avoid
    float vMax = 2.0; ///< [m/s]
    // Vertical field of view of the depthmap
    // TODO: Compute this parameters automatically.
    float vFov = 1.2822827157509358f; ///< [°]
    // Horizontal field of view of the depthmap
    // TODO: Compute this parameters automatically.
    float hFov = 1.5707963267948966f; ///< [°]
    // If the impact time between the point and the kalamos is below
    // impactTimeThreshold then it is used to compute the repulsive force
    float impactTimeThreshold = 3.0f;                     ///< [s]
    // If the avoiding speed is more than fRVinRatioThreshold percent different of
    // the input speed  and is higher than minimumAvoidSpeed then we consider that
    // we need to avoid
    float fRVinRatioThreshold = 0.1f;                     ///< [%]
    float minimumAvoidSpeed = 0.1f;                       ///< [m/s]
    // The minimum speed the drone should move at to be taken into account for
    // the
    // computation of the avoid forces. If its speed is less than this
    // threshold, it
    // is replaced by this value. In m/s.
    float minimumDroneSpeed = 1.f;                         ///< [m/s]
    float wallDetectionMinSurfacePercent = 0.7f;          ///< [%]
    float wallDetectionDistance = 0.75f;                  ///< [m]
    std::array<int, 3> pixelStopThreshold = {{8, 3, 51}}; ///< [pixel]
    std::array<int, 3> repulsiveForceFactor = {
        {3, 3, 3}};                     ///< repulsive force factor
    // When avoid is triggered the maximum roll/pitch
    // allowed to the robot is this value in degrees
    float robotMaxTilt = 8.f;            ///< [°]
    // When avoid is triggered the maximum rotation speed
    // allowed to the robot is this value in degrees per seconds
    float robotMaxRotationSpeed = 60.f;  ///< [°/s]
    // When avoid is triggered the maximum vertical speed
    // allowed to the robot is this value in meters per seconds
    float robotMaxVerticalSpeed = 0.5f; ///< [m/s]

    // Occupation window of the drone in meters
    // The drone is in the center of this window and if a point is within this
    // window then we consider it will hit the drone
    struct OccupationWindow
    {
        float height = 0.8f; ///< [m]
        float width = 0.8f;  ///< [m]
    } occupationWindow;
};

} // namespace episkopi

#endif // EPISKOPI_AVOIDSETTINGS_HPP
