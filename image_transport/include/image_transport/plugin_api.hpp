/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  Copyright (c) 2021, Austrian Institute of Technology
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
*   * Neither the name of the Willow Garage nor the names of its
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

#ifndef IMAGE_TRANSPORT__PLUGIN_API_HPP_
#define IMAGE_TRANSPORT__PLUGIN_API_HPP_

#include <memory>

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>

#include <rclcpp/node_interfaces/get_node_logging_interface.hpp>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>
#include <rclcpp/node_interfaces/get_node_parameters_interface.hpp>

#include "image_transport/visibility_control.hpp"

namespace image_transport
{

/**
 * \brief An Plugin API interface
 * 
 * This API will be constructed for and provided to plugins
 * to access any functions in order to log, create subscribers or
 * publisher and read parameters.
 * 
 * In order to not rewrite every node-api interface
 * we just provide accessors to node interfaces.
 * 
 */
class IMAGE_TRANSPORT_PUBLIC PluginApi
{
  struct PluginApiImpl;

  using ImplPtr = std::unique_ptr<PluginApiImpl>;
public:
  ~PluginApi();

  PluginApi(const PluginApi & rh);

  /**
   * \brief Takes ownership of rh
   * 
   * Do not use argument afterwards.
   */
  PluginApi(PluginApi && rh) noexcept;

  PluginApi& operator=(const PluginApi & rh);

  /**
   * \brief Takes ownership rh
   * 
   * Do not use argument afterwards.
   */
  PluginApi& operator=(PluginApi && rh) noexcept;

  /**
   * \brief Get a handle to the logging interface
   */
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& get_node_logging_interface() const& noexcept;

  /**
   * \brief Get a handle to the topics interface
   */
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& get_node_topics_interface() const& noexcept;

  /**
   * \brief Get a handle to the parameters interface
   */
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& get_node_parameters_interface() const& noexcept;

private:
  /**
   * \brief Take ownership of the passed implementation pointer
   */
  IMAGE_TRANSPORT_LOCAL
  PluginApi(ImplPtr && impl) noexcept;

  friend struct PluginApiFactory;
  
  ImplPtr impl_;
};

struct IMAGE_TRANSPORT_PUBLIC PluginApiFactory
{
  /**
   * \brief A factory for PluginApi using node interfaces
   * \return A PluginApi instance
   */
  static PluginApi create_plugin_api(
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr && logging,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr && topics,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr && parameters);
private:
  IMAGE_TRANSPORT_LOCAL
  PluginApiFactory();
};

/**
 * \brief A convenience function to construct an PluginApi from node-like types 
 * \return A PluginApi instance
 */
template<typename NodeType>
PluginApi create_plugin_api(NodeType && nh)
{
  using rclcpp::node_interfaces::get_node_logging_interface;
  using rclcpp::node_interfaces::get_node_topics_interface;
  using rclcpp::node_interfaces::get_node_parameters_interface;

  return PluginApiFactory::create_plugin_api(
    get_node_logging_interface(nh),
    get_node_topics_interface(nh),
    get_node_parameters_interface(nh));
}

}  // namespace image_transport

#endif  // IMAGE_TRANSPORT__PLUGIN_API_HPP_
