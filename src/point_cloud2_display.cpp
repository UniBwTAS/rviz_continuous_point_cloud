/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>

#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/int_property.h>
#include <rviz/validate_floats.h>

#include "point_cloud_common.h"
#include "point_cloud2_display.h"

namespace rviz
{
ContinuousPointCloud2Display::ContinuousPointCloud2Display() : point_cloud_common_(new ContinuousPointCloudCommon(this))
{
}

ContinuousPointCloud2Display::~ContinuousPointCloud2Display()
{
  ContinuousPointCloud2Display::unsubscribe();
  delete point_cloud_common_;
}

void ContinuousPointCloud2Display::onInitialize()
{
  // Use the threaded queue for processing of incoming messages
  update_nh_.setCallbackQueue(context_->getThreadedQueue());

  LLMFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void ContinuousPointCloud2Display::processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  point_cloud_common_->addMessage(cloud);
}


void ContinuousPointCloud2Display::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);
}

void ContinuousPointCloud2Display::reset()
{
  LLMFDClass::reset();
  point_cloud_common_->reset();
}

void ContinuousPointCloud2Display::updateWaitForTf()
{
  LowLatencyMessageFilterDisplay::updateWaitForTf();
  point_cloud_common_->waitForTransform(wait_for_tf_);
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::ContinuousPointCloud2Display, rviz::Display)
