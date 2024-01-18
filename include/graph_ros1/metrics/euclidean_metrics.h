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
#include <graph_ros1/metrics/metrics_base.h>
#include <graph_core/metrics/euclidean_metrics.h>
namespace graph
{
namespace ros1
{

/**
 * @class EuclideanMetrics
 * @brief This class simply implements a wrapper to graph::core::EuclideanMetrics to allow its plugin to be defined.
 */
class EuclideanMetrics: public MetricsBase //graph::ros1::MetricsBase
{
protected:
    /**
   * @brief euclidean_metrics_ Pointer to graph::core::EuclideanMetrics object, which actually performs the work.
   */
  graph::core::EuclideanMetricsPtr euclidean_metrics_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Empty constructor for EuclideanMetrics. The function MetricsBase::init() must be called afterwards.
   */
  EuclideanMetrics():MetricsBase()
  {
    euclidean_metrics_ = nullptr;
  }

  /**
   * @brief Constructs a EuclideanMetrics object.
   * @param logger A shared pointer to a TraceLogger for logging.
   */
  EuclideanMetrics(const cnr_logger::TraceLoggerPtr& logger):MetricsBase(logger) //set init_ true
  {
    euclidean_metrics_ = std::make_shared<graph::core::EuclideanMetrics>(logger);
  }

  /**
   * @brief init Initialise the object, defining its main attributes. At the end of the function, the flag 'init_' is set to true and the object can execute its main functions.
   * @param logger Pointer to a TraceLogger for logging.
   * @return True if correctly initialised, False if already initialised.
   */
  virtual bool init(const cnr_logger::TraceLoggerPtr& logger) override
  {
    return euclidean_metrics_->init(logger);
  }

  /**
   * @brief Calculates the cost between two configurations.
   * @param configuration1 The first node.
   * @param configuration2 The second node.
   * @return The cost between the two configurations.
   */
  virtual double cost(const Eigen::VectorXd& configuration1,
                      const Eigen::VectorXd& configuration2) override
  {
    return euclidean_metrics_->cost(configuration1,configuration2);
  }
  virtual double cost(const graph::core::NodePtr& node1,
                      const graph::core::NodePtr& node2) override
  {
    return cost(node1->getConfiguration(), node2->getConfiguration());
  }

  /**
   * @brief Calculates the utopia (ideal minimum cost) between two configurations.
   * @param configuration1 The first configuration.
   * @param configuration2 The second configuration.
   * @return The utopia distance between the two configurations.
   */
  virtual double utopia(const Eigen::VectorXd& configuration1,
                        const Eigen::VectorXd& configuration2) override
  {
    return euclidean_metrics_->utopia(configuration1,configuration2);
  }
  virtual double utopia(const graph::core::NodePtr& node1,
                        const graph::core::NodePtr& node2) override
  {
    return utopia(node1->getConfiguration(), node2->getConfiguration());
  }

  /**
   * @brief Creates a clone of the EuclideanMetrics object.
   * @return A shared pointer to the cloned EuclideanMetrics object.
   */
  virtual graph::core::MetricsPtr clone() override
  {
    return std::make_shared<EuclideanMetrics>(logger_);
  }
};

} //namespace ros1
} //namespace graph
