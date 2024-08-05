#pragma once
/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, UNIBS and CNR-STIIMA, manuel.beschi@unibs.it, c.tonola001@unibs.it
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
#include <cnr_class_loader/register_macro.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <graph_core/collision_checkers/collision_checker_base.h>
#include <moveit_collision_checker/collision_checkers/moveit_collision_checker.h>

namespace graph
{
namespace collision_check
{

/**
 * @class MoveitCollisionCheckerBasePlugin
 * @brief This class implements a base class wrapper for collision checker plugins implemented using Moveit.
 * The class can be loaded as a plugin and builds a graph::core::CollisionCheckerBase object.
 */
class MoveitCollisionCheckerBasePlugin;
typedef std::shared_ptr<MoveitCollisionCheckerBasePlugin> MoveitCollisionCheckerPluginPtr;

class MoveitCollisionCheckerBasePlugin: std::enable_shared_from_this<MoveitCollisionCheckerBasePlugin>
{
protected:

  /**
   * @brief collision_checker_ is the graph::collision_check::MoveitCollisionCheckerPtr object built and initialized by this plugin class.
   */
  graph::collision_check::MoveitCollisionCheckerPtr collision_checker_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for MoveitCollisionCheckerBasePlugin. The function init() must be called afterwards.
   */
  MoveitCollisionCheckerBasePlugin()
  {
    collision_checker_ = nullptr;
  }

  /**
   * @brief Destructor for MoveitCollisionCheckerBasePlugin.
   */
  virtual ~MoveitCollisionCheckerBasePlugin()
  {
    collision_checker_ = nullptr;
  }

  /**
   * @brief getCollisionChecker return the graph::collision_check::MoveitCollisionCheckerPtr object built by the plugin.
   * @return the graph::collision_check::MoveitCollisionCheckerPtr object built.
   */
  graph::collision_check::MoveitCollisionCheckerPtr getCollisionChecker()
  {
    return collision_checker_;
  }

  /**
   * @brief init Initialise the graph::collision_check::MoveitCollisionCheckerPtr object, defining its main attributes.
   * @param param_ns defines the namespace under which parameter are searched for using cnr_param library. MoveitCollisionChecker requires group_name and checker_resolution as parameters.
   * @param planning_scene Pointer to the MoveIt! PlanningScene.
   * @param logger Pointer to a TraceLogger for logging.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const std::string& param_ns,
                    const planning_scene::PlanningScenePtr& planning_scene,
                    const cnr_logger::TraceLoggerPtr& logger)=0;
};

} //namespace collision_check
} //namespace graph
