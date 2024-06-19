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

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <graph_core/collision_checkers/collision_checker_base.h>

namespace graph
{

using namespace graph::core;

namespace collision_check
{

/**
 * @class MoveitCollisionChecker
 * @brief Implementation of CollisionCheckerBase using MoveIt! library for collision checking.
 *
 * This class inherits from CollisionCheckerBase and provides collision checking functionality
 * using the MoveIt! library. It interacts with a MoveIt! PlanningScene to perform collision checks
 * for a specified joint group.
 */
class MoveitCollisionChecker: public CollisionCheckerBase //graph::core::CollisionCheckerBase
{
protected:
  /**
   * @brief planning_scene_ Pointer to the MoveIt! PlanningScene.
   */
  planning_scene::PlanningScenePtr planning_scene_;

  /**
   * @brief state_ Pointer to the current state of the robot (moveit::core::RobotState) used for collision checking.
   */
  moveit::core::RobotStatePtr state_;

  /**
   * @brief group_name_  Name of the joint group to consider for collision checking.
   */
  std::string group_name_;

  /**
   * @brief jmg_ Pointer to the group of joints of the robot identified by the group name.
   */
  const moveit::core::JointModelGroup* jmg_;

  /**
   * @brief joint_names_ Names of active joints in the group.
   */
  std::vector<std::string> joint_names_;

  /**
   * @brief joint_models_ Pointers to active joints in the group.
   */
  std::vector<const moveit::core::JointModel*> joint_models_;

  /**
   * @brief mimic_joint_models_ Pointers to mimic joints in the group.
   * Mimic joints are typically unactuated joints that are constrained to follow the motion of another joint.
   */
  std::vector<const moveit::core::JointModel*> mimic_joint_models_;

  /**
   * @brief Pointer to a TraceLogger instance for logging.
   *
   * This member variable represents a pointer to a TraceLogger instance, allowing
   * to perform logging operations. TraceLogger is a part of the cnr_logger library.
   * Ensure that the logger is properly configured and available for use.
   */
  cnr_logger::TraceLoggerPtr logger_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for MoveitCollisionChecker. The function init() must be called afterwards.
   */
  MoveitCollisionChecker():CollisionCheckerBase(){} //set init_ false

  /**
   * @brief Constructor for MoveitCollisionChecker class.
   *
   * @param planning_scene Pointer to the MoveIt! PlanningScene.
   * @param group_name Name of the joint group for collision checking.
   * @param logger Pointer to the logger for logging messages.
   * @param min_distance Minimum distance for collision checking (default is 0.01).
   */
  MoveitCollisionChecker(const planning_scene::PlanningScenePtr& planning_scene,
                         const std::string& group_name,
                         const cnr_logger::TraceLoggerPtr& logger,
                         const double& min_distance = 0.01):
    CollisionCheckerBase(logger,min_distance), //set init_ true
    planning_scene_(planning_scene),
    group_name_(group_name),
    logger_(logger)
  {
    state_ = std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());
    jmg_ = state_->getJointModelGroup(group_name_);
    joint_names_=jmg_->getActiveJointModelNames();
    joint_models_=jmg_->getActiveJointModels();
    mimic_joint_models_=jmg_->getMimicJointModels();

    if (!planning_scene_)
      CNR_ERROR(logger_,"Invalid planning scene");
  }

  /**
   * @brief init Initialise the object, defining its main attributes. At the end of the function, the flag 'init_' is set to true and the object can execute its main functions.
   * @param planning_scene Pointer to the MoveIt! PlanningScene.
   * @param group_name Name of the joint group for collision checking.
   * @param logger Pointer to the logger for logging messages.
   * @param min_distance Minimum distance for collision checking (default is 0.01).
   */
  virtual bool init(const planning_scene::PlanningScenePtr& planning_scene,
                    const std::string& group_name,
                    const cnr_logger::TraceLoggerPtr& logger,
                    const double& min_distance = 0.01)
  {
    if(not CollisionCheckerBase::init(logger,min_distance))
      return false;

    planning_scene_ = planning_scene;
    group_name_ = group_name;

    state_ = std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());
    jmg_ = state_->getJointModelGroup(group_name_);
    joint_names_=jmg_->getActiveJointModelNames();
    joint_models_=jmg_->getActiveJointModels();
    mimic_joint_models_=jmg_->getMimicJointModels();

    if (!planning_scene_)
      CNR_ERROR(logger_,"Invalid planning scene");

    return true;
  }

  /**
   * @brief Set the MoveIt! PlanningScene based on a PlanningScene message.
   *
   * @param msg The PlanningScene message.
   */
  virtual void setPlanningSceneMsg(const moveit_msgs::msg::PlanningScene& msg)
  {
    if (!planning_scene_->usePlanningSceneMsg(msg))
      CNR_ERROR_THROTTLE(logger_,1,"Unable to upload scene");
  }

  /**
   * @brief Clone the CollisionChecker.
   *
   * @return A new indipendent CollisionCheckerPtr with the same configuration.
   */
  virtual CollisionCheckerPtr clone() override
  {
    planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scene_);
    return std::make_shared<MoveitCollisionChecker>(planning_scene,group_name_,logger_,min_distance_);
  }

  /**
   * @brief Perform collision checking for a given joint configuration.
   *
   * @param configuration The joint configuration to check for collisions.
   * @return True if collision-free, false if collision occurs.
   */
  virtual bool check(const Eigen::VectorXd& configuration) override
  {
    *state_ = planning_scene_->getCurrentState();

    for (size_t ij=0;ij<joint_names_.size();ij++)
      state_->setJointPositions(joint_models_.at(ij),&configuration(ij));


    if (!state_->satisfiesBounds(jmg_))
    {
      CNR_DEBUG(logger_,"Out of bound");
      return false;
    }
    state_->update();
    state_->updateCollisionBodyTransforms();
    return !planning_scene_->isStateColliding(*state_,group_name_);
  }

  /**
   * @brief Set the MoveIt! PlanningScene.
   *
   * @param planning_scene Pointer to the new MoveIt! PlanningScene.
   */
  virtual void setPlanningScene(planning_scene::PlanningScenePtr &planning_scene)
  {
    planning_scene_ = planning_scene;
  }

  /**
   * @brief Get the name of the joint group.
   *
   * @return The name of the joint group.
   */
  std::string getGroupName() override
  {
    return group_name_;
  }

  /**
   * @brief Get the MoveIt! PlanningScene instance used by the collision checker.
   *
   * @return Pointer to the MoveIt! PlanningScene.
   */
  planning_scene::PlanningScenePtr getPlanningScene()
  {
    return planning_scene_;
  }

};

} //namespace collision_check
} //namespace graph
