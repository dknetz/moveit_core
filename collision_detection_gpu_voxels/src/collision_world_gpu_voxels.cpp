/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Daniel Kuhner */

#include <moveit/collision_detection_gpu_voxels/collision_world_gpu_voxels.h>

collision_detection::CollisionWorldGpuVoxels::CollisionWorldGpuVoxels()
{
}

collision_detection::CollisionWorldGpuVoxels::CollisionWorldGpuVoxels(const WorldPtr& world)
{
}

collision_detection::CollisionWorldGpuVoxels::CollisionWorldGpuVoxels(const CollisionWorldGpuVoxels& other,
		const WorldPtr& world)
{
}

collision_detection::CollisionWorldGpuVoxels::~CollisionWorldGpuVoxels()
{
}

void collision_detection::CollisionWorldGpuVoxels::checkRobotCollision(const CollisionRequest& req,
		CollisionResult& res,
		const CollisionRobot& robot,
		const robot_state::RobotState& state) const
		{
	res.collision = false;
}

void collision_detection::CollisionWorldGpuVoxels::checkRobotCollision(const CollisionRequest& req,
		CollisionResult& res,
		const CollisionRobot& robot,
		const robot_state::RobotState& state,
		const AllowedCollisionMatrix& acm) const
		{
	res.collision = false;
}

void collision_detection::CollisionWorldGpuVoxels::checkRobotCollision(const CollisionRequest& req,
		CollisionResult& res,
		const CollisionRobot& robot,
		const robot_state::RobotState& state1,
		const robot_state::RobotState& state2) const
		{
	res.collision = false;
}

void collision_detection::CollisionWorldGpuVoxels::checkRobotCollision(const CollisionRequest& req,
		CollisionResult& res,
		const CollisionRobot& robot,
		const robot_state::RobotState& state1,
		const robot_state::RobotState& state2,
		const AllowedCollisionMatrix& acm) const
		{
	res.collision = false;
}

void collision_detection::CollisionWorldGpuVoxels::checkWorldCollision(const CollisionRequest& req,
		CollisionResult& res,
		const CollisionWorld& other_world) const
		{
	res.collision = false;
}

void collision_detection::CollisionWorldGpuVoxels::checkWorldCollision(const CollisionRequest& req,
		CollisionResult& res,
		const CollisionWorld& other_world,
		const AllowedCollisionMatrix& acm) const
		{
	res.collision = false;
}

double collision_detection::CollisionWorldGpuVoxels::distanceRobot(const CollisionRobot& robot,
		const robot_state::RobotState& state) const
		{
	return 100;
}

double collision_detection::CollisionWorldGpuVoxels::distanceRobot(const CollisionRobot& robot,
		const robot_state::RobotState& state,
		const AllowedCollisionMatrix& acm) const
		{
	return 100;
}

double collision_detection::CollisionWorldGpuVoxels::distanceWorld(const CollisionWorld& world) const
		{
	return 100;
}

double collision_detection::CollisionWorldGpuVoxels::distanceWorld(const CollisionWorld& world,
		const AllowedCollisionMatrix& acm) const
		{
	return 100;
}

void collision_detection::CollisionWorldGpuVoxels::setWorld(const WorldPtr& world)
{
}

#include "../include/moveit/collision_detection_gpu_voxels/collision_detector_allocator_gpu_voxels.h"
const std::string collision_detection::CollisionDetectorAllocatorGPUVoxels::NAME_("gpu-voxels");
