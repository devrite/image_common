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
#include "image_transport/plugin_api.hpp"

namespace image_transport
{

/**
 * \brief A class for storing and accessing relevant interfaces
 * 
 * Hidden to plugins in order modify or extend as needed.
 */
struct IMAGE_TRANSPORT_LOCAL PluginApi::PluginApiImpl
{
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;

  PluginApiImpl() noexcept = default;

  /**
   * \brief Constructor taking ownership of the interfaces
   */
  PluginApiImpl(
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr && nl,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr && nt,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr && np)
  noexcept
  : logging{std::move(nl)},
    topics{std::move(nt)},
    parameters{std::move(np)}
  {
  }
  friend struct PluginApiFactory;
};

PluginApi::PluginApi(ImplPtr && impl) noexcept
: impl_{std::move(impl)}
{
}

PluginApi::PluginApi(const PluginApi & rh)
: impl_{ std::make_unique<PluginApiImpl>()}
{
  *impl_ = *rh.impl_;
}


PluginApi::PluginApi(PluginApi && rh) noexcept
: impl_{std::move(rh.impl_)}
{
}

PluginApi& PluginApi::operator=(const PluginApi & rh)
{
  *impl_ = *rh.impl_;
  return *this;
}

PluginApi& PluginApi::operator=(PluginApi && rh) noexcept
{
  rh.impl_.swap(impl_);
  return *this;
}

PluginApi::~PluginApi() = default;

const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& PluginApi::get_node_logging_interface() const& noexcept
{
  return impl_->logging;
}

const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& PluginApi::get_node_topics_interface() const& noexcept
{
  return impl_->topics;
}

const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& PluginApi::get_node_parameters_interface() const& noexcept
{
  return impl_->parameters;
}

PluginApi PluginApiFactory::create_plugin_api(
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr && logging,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr && topics,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr && parameters)
{
  return PluginApi(std::make_unique<PluginApi::PluginApiImpl>(
    std::move(logging),
    std::move(topics),
    std::move(parameters)));
}

}  // namespace image_transport
