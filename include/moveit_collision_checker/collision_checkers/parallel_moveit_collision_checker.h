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


#include <moveit_collision_checker/collision_checkers/moveit_collision_checker.h>
#include <thread>
#include <future>
#include <mutex>

namespace graph
{

using namespace graph::core;

namespace ros2
{

/**
 * @class ParallelMoveitCollisionChecker
 * @brief Implementation of a parallel collision checker using MoveIt! library for multi-threaded collision checking.
 *
 * This class extends the MoveitCollisionChecker and adds parallel collision checking functionality
 * by distributing collision checks across multiple threads.
 */
class ParallelMoveitCollisionChecker: public MoveitCollisionChecker
{
#define GROUP_SIZE 10

protected:

  /**
   * @brief threads_num_   Number of parallel threads for collision checking.
   */
  int threads_num_;

  /**
   * @brief thread_iter_ Iterator for distributing collision checks among threads.
   */
  int thread_iter_=0;

  /**
   * @brief stop_check_ Flag to stop ongoing collision checks.
   */
  bool stop_check_;

  /**
   * @brief at_least_a_collision_ Flag indicating if at least one collision is detected.
   */
  bool at_least_a_collision_;

  /**
   * @brief queues_ Queues for storing joint configurations to be checked.
   */
  std::vector<std::vector<std::vector<double>>> queues_;

  /**
   * @brief threads_ Vector of threads for parallel collision checking.
   */
  std::vector<std::thread> threads_;

  /**
   * @brief planning_scenes_ Vector of PlanningScene objects for each thread.
   */
  std::vector<planning_scene::PlanningScenePtr> planning_scenes_;

  /**
   * @brief Reset the queue storing the configurations to check for each thread.
   */
  void resetQueue();

  /**
   * @brief Add a joint configuration to the queues for collision checking.
   *
   * @param q Joint configuration to be added to the queue.
   */
  void queueUp(const Eigen::VectorXd &q);

  /**
   * @brief Check collisions for all joint configurations in the queues.
   *
   * @return True if all configurations are collision-free, false if at least one collision is detected.
   */
  bool checkAllQueues();

  /**
   * @brief Collision checking thread function for each parallel thread.
   *
   * @param thread_idx Index of the thread.
   */
  void collisionThread(int thread_idx);

  /**
   * @brief Asynchronously set the PlanningScene based on a PlanningScene message for a group of threads.
   *
   * @param msg The PlanningScene message.
   * @param idx Index of the first thread to update.
   * @return True if successful, false otherwise.
   */
  bool asyncSetPlanningSceneMsg(const moveit_msgs::msg::PlanningScene& msg, const int &idx);

  /**
   * @brief Asynchronously clone and set the PlanningScene for a group of threads.
   *
   * @param scene Pointer to the new PlanningScene.
   * @param idx Index of the first thread to update.
   * @return True if successful, false otherwise.
   */
  bool asyncSetPlanningScene(const planning_scene::PlanningScenePtr& scene, const int& idx);

  /**
   * @brief Add to the queues the joint configurations along the connection between two given configurations.
   *
   * @param configuration1 Starting joint configuration.
   * @param configuration2 Ending joint configuration.
   */
  void queueConnection(const Eigen::VectorXd& configuration1,
                       const Eigen::VectorXd& configuration2);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for ParallelMoveitCollisionChecker. The function init() must be called afterwards.
   */
  ParallelMoveitCollisionChecker():MoveitCollisionChecker(){}

  /**
   * @brief Constructor for ParallelMoveitCollisionChecker class.
   *
   * @param planning_scene Pointer to the MoveIt! PlanningScene.
   * @param group_name Name of the joint group for collision checking.
   * @param logger Pointer to the logger for logging messages.
   * @param threads_num Number of parallel tros1hreads for collision checking (default is 4).
   * @param min_distance Minimum distance for collision checking (default is 0.01).
   */
  ParallelMoveitCollisionChecker(const planning_scene::PlanningScenePtr& planning_scene,
                                 const std::string& group_name,
                                 const cnr_logger::TraceLoggerPtr& logger,
                                 const int& threads_num=4,
                                 const double& min_distance = 0.01);

  /**
   * @brief Destructor for ParallelMoveitCollisionChecker class.
   */
  ~ParallelMoveitCollisionChecker();

  /**
   * @brief init Initialise the object, defining its main attributes. At the end of the function, the flag 'init_' is set to true and the object can execute its main functions.
   *
   * @param planning_scene Pointer to the MoveIt! PlanningScene.
   * @param group_name Name of the joint group for collision checking.
   * @param logger Pointer to the logger for logging messages.
   * @param threads_num Number of parallel threads for collision checking (default is 4).
   * @param min_distance Minimum distance for collision checking (default is 0.01).
   */
  virtual bool init(const planning_scene::PlanningScenePtr& planning_scene,
                    const std::string& group_name,
                    const cnr_logger::TraceLoggerPtr& logger,
                    const int& threads_num=4,
                    const double& min_distance = 0.0);

  /**
   * @brief Set the PlanningScene based on a PlanningScene message for the entire group of threads.
   *
   * @param msg The PlanningScene message.
   */
  virtual void setPlanningSceneMsg(const moveit_msgs::msg::PlanningScene& msg) override;

  /**
   * @brief Set the PlanningScene for the entire group of threads.
   *
   * @param planning_scene Pointer to the new PlanningScene.
   */
  virtual void setPlanningScene(planning_scene::PlanningScenePtr &planning_scene) override;

  /**
   * @brief Check collisions along a path between two joint configurations.
   *
   * @param configuration1 Starting joint configuration.
   * @param configuration2 Ending joint configuration.
   * @return True if the path is collision-free, false if collisions are detected.
   */
  virtual bool checkConnection(const Eigen::VectorXd& configuration1,
                         const Eigen::VectorXd& configuration2) override;

  virtual bool checkConnFromConf(const ConnectionPtr& conn,
                                 const Eigen::VectorXd& this_conf) override;

  virtual bool checkConnections(const std::vector<ConnectionPtr>& connections) override;

  virtual CollisionCheckerPtr clone() override;

};

} //namespace ros2
} //namespace graph
