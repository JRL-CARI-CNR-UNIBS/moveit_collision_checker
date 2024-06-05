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

#include <moveit_collision_checker/plugins/collision_checkers/moveit_collision_checker_base_plugin.h>
#include <moveit_collision_checker/collision_checkers/moveit_collision_checker.h>

namespace graph
{
namespace ros2
{

/**
 * @class MoveitCollisionCheckerPlugin
 * @brief This class implements a wrapper to graph::ros2::MoveitCollisionChecker to allow its plugin to be defined.
 */
class MoveitCollisionCheckerPlugin: public MoveitCollisionCheckerBasePlugin
{
protected:

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for MoveitCollisionCheckerPlugin. The function MoveitCollisionCheckerPlugin::init() must be called afterwards.
   */
  MoveitCollisionCheckerPlugin():MoveitCollisionCheckerBasePlugin()
  {}

  /**
   * @brief init Initialise the graph::ros2::MoveitCollisionChecker object, defining its main attributes.
   * @param param_ns defines the namespace under which parameter are searched for using cnr_param library.
   * @param planning_scene Pointer to the MoveIt! PlanningScene.
   * @param logger Pointer to a TraceLogger for logging.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const std::string& param_ns,
                    const planning_scene::PlanningScenePtr& planning_scene,
                    const cnr_logger::TraceLoggerPtr& logger) override
  {
    double checker_resolution;
    graph::core::get_param(logger,param_ns,"checker_resolution",checker_resolution,0.01);

    std::string group_name;
    graph::core::get_param(logger,param_ns,"group_name",group_name,(std::string)"manipulator");

    collision_checker_ = std::make_shared<graph::ros2::MoveitCollisionChecker>(planning_scene,group_name,logger,checker_resolution);

    return true;
  }

};

} //namespace ros2
} //namespace graph
