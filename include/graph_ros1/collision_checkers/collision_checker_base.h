#pragma once
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <graph_core/collision_checkers/collision_checker_base.h>

namespace graph
{
namespace ros1
{

/**
 * @class CollisionCheckerBase
 * @brief This class simply derive from graph::core::CollisionCheckerBase to allow the implementation of plugins with CollisionCheckerBase as base class.
 *
 */
class CollisionCheckerBase;
typedef std::shared_ptr<CollisionCheckerBase> CollisionCheckerPtr;

class CollisionCheckerBase: public graph::core::CollisionCheckerBase
{
protected:

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for CollisionCheckerBase. The function init() must be called afterwards.
   */
  CollisionCheckerBase(): graph::core::CollisionCheckerBase(){}

  /**
   * @brief Constructor for CollisionCheckerBase.
   * @param logger Pointer to a TraceLogger for logging.
   * @param min_distance Distance between configurations checked for collisions along a connection.
   */
  CollisionCheckerBase(const cnr_logger::TraceLoggerPtr& logger, const double& min_distance = 0.01):
    graph::core::CollisionCheckerBase(logger,min_distance){}

};

} //namespace ros1
} //namespace graph
