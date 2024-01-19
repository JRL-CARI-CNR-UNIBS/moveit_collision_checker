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

#include <graph_ros1/plugins/collision_checkers/collision_checker_base_plugin.h>
#include <graph_ros1/collision_checkers/moveit_collision_checker.h>

namespace graph
{
namespace ros1
{

/**
 * @class MoveitCollisionCheckerPlugin
 * @brief This class implements a wrapper to graph::ros1::MoveitCollisionChecker to allow its plugin to be defined.
 */
class MoveitCollisionCheckerPlugin: public CollisionCheckerBasePlugin
{
protected:

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for MoveitCollisionCheckerPlugin. The function MoveitCollisionCheckerPlugin::init() must be called afterwards.
   */
  MoveitCollisionCheckerPlugin():CollisionCheckerBasePlugin()
  {}

  /**
   * @brief init Initialise the graph::ros1::MoveitCollisionChecker object, defining its main attributes.
   * @param nh Ros node handle to read params from the ros parameters server. MoveitCollisionChecker requires group_name and collision_checker_min_step as parameters.
   * @param planning_scene Pointer to the MoveIt! PlanningScene.
   * @param logger Pointer to a TraceLogger for logging.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const ros::NodeHandle& nh,
                    const planning_scene::PlanningScenePtr& planning_scene,
                    const cnr_logger::TraceLoggerPtr& logger) override
  {
    double collision_checker_min_step = 0.01;
    std::string group_name = "manipulator";

    if(not nh.getParam("collision_checker_min_step", collision_checker_min_step))
      CNR_WARN(logger,"collision_checker_min_step not defined, using "<<collision_checker_min_step);

    if(not nh.getParam("group_name", group_name))
      CNR_WARN(logger,"group_name not defined, using "<<group_name);

    collision_checker_ = std::make_shared<graph::ros1::MoveitCollisionChecker>(planning_scene,group_name,logger,collision_checker_min_step);

    return true;
  }

};

} //namespace ros1
} //namespace graph
