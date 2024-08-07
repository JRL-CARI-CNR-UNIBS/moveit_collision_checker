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

#include <moveit_collision_checker/collision_checkers/parallel_moveit_collision_checker.h>

namespace graph
{
namespace collision_check
{

ParallelMoveitCollisionChecker::ParallelMoveitCollisionChecker(const planning_scene::PlanningScenePtr &planning_scene,
                                                                   const std::string& group_name,
                                                                   const cnr_logger::TraceLoggerPtr& logger,
                                                                   const int& threads_num,
                                                                   const double& min_distance):
  MoveitCollisionChecker(planning_scene,group_name,logger,min_distance),
  threads_num_(threads_num)
{
  if (threads_num<=0)
  {
    CNR_FATAL(logger_,"number of thread should be positive");
    throw std::invalid_argument("number of thread should be positive");
  }

  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;

  queues_.clear();
  planning_scenes_.clear();

  for (int idx=0;idx<threads_num_;idx++)
  {
    planning_scenes_.push_back(planning_scene::PlanningScene::clone(planning_scene_));
    queues_.push_back(std::vector<std::vector<double>>());
  }
  
  pool_ = std::make_shared<BS::thread_pool>(threads_num_);

}

bool ParallelMoveitCollisionChecker::init(const planning_scene::PlanningScenePtr& planning_scene,
                                            const std::string& group_name,
                                            const cnr_logger::TraceLoggerPtr& logger,
                                            const int& threads_num,
                                            const double& min_distance)
{
  if(not MoveitCollisionChecker::init(planning_scene,group_name,logger,min_distance))
    return false;

  if (threads_num<=0)
  {
    CNR_FATAL(logger_,"number of thread should be positive");
    throw std::invalid_argument("number of thread should be positive");
  }
  else
    threads_num_ = threads_num;

  at_least_a_collision_=false;
  stop_check_=true;
  thread_iter_=0;

  queues_.clear();
  planning_scenes_.clear();

  for (int idx=0;idx<threads_num_;idx++)
  {
    planning_scenes_.push_back(planning_scene::PlanningScene::clone(planning_scene_));
    queues_.push_back(std::vector<std::vector<double>>());
  }

  pool_ = std::make_shared<BS::thread_pool>(threads_num_);

  return true;
}

void ParallelMoveitCollisionChecker::resetQueue()
{
  at_least_a_collision_=false;
  stop_check_=true;

  thread_iter_=0;
  pool_->wait();
  assert(pool_->get_tasks_total  () == 0);
  assert(pool_->get_tasks_queued () == 0);
  assert(pool_->get_tasks_running() == 0);

  pool_->purge();

  for (int idx=0;idx<threads_num_;idx++)
    queues_.at(idx).clear();
}

void ParallelMoveitCollisionChecker::queueUp(const Eigen::VectorXd &q)
{
  if(q.size() != 0)
  {
    std::vector<double> conf(q.size());
    for (unsigned idx=0;idx<q.size();idx++)
      conf.at(idx)=q(idx);

    queues_.at(thread_iter_).push_back(conf);
    thread_iter_ ++;
    if (thread_iter_>=threads_num_)
      thread_iter_=0;
  }
  else
  {
    CNR_FATAL(logger_,"q is empty");
    throw std::invalid_argument("q is empty");
  }
}

bool ParallelMoveitCollisionChecker::checkAllQueues()
{
  stop_check_=false;

  for (int idx=0;idx<threads_num_;idx++)
  {
    if(queues_.at(idx).size()>0)
    {
      pool_->detach_task(
            [idx,this]
      {
        this->collisionThread(idx);
      });
    }
  }

  pool_->wait();
  assert(pool_->get_tasks_total  () == 0);
  assert(pool_->get_tasks_queued () == 0);
  assert(pool_->get_tasks_running() == 0);

  return !at_least_a_collision_;
}

void ParallelMoveitCollisionChecker::collisionThread(int thread_idx)
{
  robot_state::RobotStatePtr state=std::make_shared<robot_state::RobotState>(planning_scenes_.at(thread_idx)->getCurrentState());
  const std::vector<std::vector<double>>& queue=queues_.at(thread_idx);
  for (const std::vector<double>& configuration: queue)
  {
    if(stop_check_)
      break;

    assert(configuration.size()>0);

    for (size_t ij=0;ij<joint_names_.size();ij++)
    {
      state->setJointPositions(joint_models_.at(ij),&configuration.at(ij));
    }

    state->update();
    //    state->updateCollisionBodyTransforms(); //update() already calls it

    if (!state->satisfiesBounds(jmg_))
    {
      at_least_a_collision_=true;
      stop_check_=true;
      break;
    }
    
    if (planning_scenes_.at(thread_idx)->isStateColliding(*state,group_name_))
    {
      at_least_a_collision_=true;
      stop_check_=true;
      break;
    }
  }
}

ParallelMoveitCollisionChecker::~ParallelMoveitCollisionChecker()
{
  CNR_DEBUG(logger_,"Closing collision threads");
  stop_check_=true;

  pool_->wait();
  assert(pool_->get_tasks_total  () == 0);
  assert(pool_->get_tasks_queued () == 0);
  assert(pool_->get_tasks_running() == 0);

  pool_->purge();

  CNR_DEBUG(logger_,"Collision threads closed");
}

bool ParallelMoveitCollisionChecker::asyncSetPlanningSceneMsg(const moveit_msgs::PlanningScene& msg, const int& idx)
{
  bool res = true;
  for(int i=idx;i<(idx+GROUP_SIZE);i++)
  {
    if(i == threads_num_)
      break;

    if(not planning_scenes_[i]->usePlanningSceneMsg(msg))
    {
      CNR_ERROR_THROTTLE(logger_,1,"Unable to upload scene");
      res = false;
    }
  }

  return res;
}

bool ParallelMoveitCollisionChecker::asyncSetPlanningScene(const planning_scene::PlanningScenePtr& scene, const int& idx)
{
  for(int i=idx;i<(idx+GROUP_SIZE);i++)
  {
    if(i == threads_num_)
      break;

    planning_scenes_[i]=planning_scene::PlanningScene::clone(scene);
  }

  return true;
}

void ParallelMoveitCollisionChecker::setPlanningSceneMsg(const moveit_msgs::PlanningScene& msg)
{
  stop_check_=true;

  pool_->wait();
  assert(pool_->get_tasks_total  () == 0);
  assert(pool_->get_tasks_queued () == 0);
  assert(pool_->get_tasks_running() == 0);

  if (!planning_scene_->usePlanningSceneMsg(msg))
    CNR_ERROR_THROTTLE(logger_,1,"Unable to upload scene");

  int n_groups = std::floor(threads_num_/GROUP_SIZE);
  if(threads_num_%GROUP_SIZE == 0 && n_groups != 0)
    n_groups = n_groups-1;

  int group_start = 0;
  std::vector<std::shared_future<bool>> futures;
  for (int idx=0;idx<n_groups;idx++)
  {
    futures.push_back(std::async(std::launch::async,&ParallelMoveitCollisionChecker::asyncSetPlanningSceneMsg,this,msg,group_start));
    group_start = group_start+GROUP_SIZE;
  }

  for(int idx=group_start;idx<threads_num_;idx++)
  {
    planning_scenes_[idx]->usePlanningSceneMsg(msg);
  }

  for (int idx=0;idx<n_groups;idx++)
  {
    if(!futures.at(idx).get())
      CNR_ERROR_THROTTLE(logger_,1,"unable to upload scene");
  }
}

void ParallelMoveitCollisionChecker::setPlanningScene(planning_scene::PlanningScenePtr &planning_scene)
{
  stop_check_=true;

  pool_->wait();
  assert(pool_->get_tasks_total  () == 0);
  assert(pool_->get_tasks_queued () == 0);
  assert(pool_->get_tasks_running() == 0);

  planning_scene_ = planning_scene;

  int n_groups = std::floor(threads_num_/GROUP_SIZE);
  if(threads_num_%GROUP_SIZE == 0 && n_groups != 0)
    n_groups = n_groups-1;

  int group_start = 0;
  std::vector<std::shared_future<bool>> futures;
  for (int idx=0;idx<n_groups;idx++)
  {
    futures.push_back(std::async(std::launch::async,&ParallelMoveitCollisionChecker::asyncSetPlanningScene,this,planning_scene,group_start));
    group_start = group_start+GROUP_SIZE;
  }

  for(int idx=group_start;idx<threads_num_;idx++)
  {
    planning_scenes_[idx]=planning_scene::PlanningScene::clone(planning_scene);
  }

  for (int idx=0;idx<n_groups;idx++)
  {
    if(!futures.at(idx).get())
      CNR_ERROR_THROTTLE(logger_,1,"Unable to upload scene");
  }
}

void ParallelMoveitCollisionChecker::queueConnection(const Eigen::VectorXd& configuration1,
                                                       const Eigen::VectorXd& configuration2)
{
  double distance = (configuration2 - configuration1).norm();
  if (distance < min_distance_)
    return;

  Eigen::VectorXd conf(configuration1.size());
  double n = 2;

  while (distance > n * min_distance_)
  {
    for (double idx = 1; idx < n; idx += 2)
    {
      conf = configuration1 + (configuration2 - configuration1) * idx / n;
      queueUp(conf);
    }
    n *= 2;
  }
}

bool ParallelMoveitCollisionChecker::checkConnection(const Eigen::VectorXd& configuration1,
                                                       const Eigen::VectorXd& configuration2)
{
  resetQueue();
  if (!check(configuration1))
    return false;
  if (!check(configuration2))
    return false;
  queueConnection(configuration1,configuration2);
  return checkAllQueues();
}

bool ParallelMoveitCollisionChecker::checkConnFromConf(const ConnectionPtr& conn,
                                                         const Eigen::VectorXd& this_conf)
{
  resetQueue();
  Eigen::VectorXd parent = conn->getParent()->getConfiguration();
  Eigen::VectorXd child = conn->getChild()->getConfiguration();

  double dist_child = (this_conf-child).norm();
  double dist_parent = (parent-this_conf).norm();
  double dist = (parent-child).norm();

  if((dist-dist_child-dist_parent)>1e-04)
  {
    CNR_FATAL(logger_,"The conf is not on the connection between parent and child");
    throw std::invalid_argument("The conf is not on the connection between parent and child");
  }

  if(!check(this_conf)) return false;
  if(!check(child)) return false;

  double distance = (this_conf - child).norm();
  if(distance < min_distance_)
  {
    return true;
  }

  double this_abscissa = (parent-this_conf).norm()/(parent-child).norm();
  double abscissa;
  double n = 2;
  Eigen::VectorXd conf;
  while (distance > n * min_distance_)
  {
    for (double idx = 1; idx < n; idx += 2)
    {
      abscissa = idx/n;
      if(abscissa>=this_abscissa)
      {
        conf = parent + (child - parent) * abscissa;
        queueUp(conf);
      }
    }
    n *= 2;
  }

  return checkAllQueues();
}

bool ParallelMoveitCollisionChecker::checkConnections(const std::vector<ConnectionPtr>& connections)
{
  resetQueue();
  if (!check(connections.front()->getParent()->getConfiguration()))
    return false;
  for (const ConnectionPtr& c: connections)
  {
    if (!check(c->getChild()->getConfiguration()))
      return false;
    queueConnection(c->getParent()->getConfiguration(),c->getChild()->getConfiguration());
  }
  return checkAllQueues();
}

CollisionCheckerPtr ParallelMoveitCollisionChecker::clone()
{
  planning_scene::PlanningScenePtr planning_scene = planning_scene::PlanningScene::clone(planning_scene_);
  return std::make_shared<ParallelMoveitCollisionChecker>(planning_scene,group_name_,logger_,threads_num_,min_distance_);
}
} //namespace collision_check
} //namespace graph
