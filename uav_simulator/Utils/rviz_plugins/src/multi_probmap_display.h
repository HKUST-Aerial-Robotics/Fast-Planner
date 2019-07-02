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

#ifndef MULTI_PROB_MAP_DISPLAY_H
#define MULTI_PROB_MAP_DISPLAY_H

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreVector3.h>

#include <nav_msgs/MapMetaData.h>
#include <ros/time.h>

#include <multi_map_server/MultiOccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>

#include "rviz/display.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz
{

class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class RosTopicProperty;
class VectorProperty;

/**
 * \class MultiProbMapDisplay
 * \brief Displays a map along the XY plane.
 */
class MultiProbMapDisplay : public Display
{
  Q_OBJECT
public:
  MultiProbMapDisplay();
  virtual ~MultiProbMapDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void reset();
  virtual void update(float wall_dt, float ros_dt);

protected Q_SLOTS:
  void updateTopic();
  void updateDrawUnder();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  void incomingMap(const multi_map_server::MultiOccupancyGrid::ConstPtr& msg);

  void clear();

  std::vector<Ogre::ManualObject*> manual_object_;
  std::vector<Ogre::TexturePtr>    texture_;
  std::vector<Ogre::MaterialPtr>   material_;

  bool loaded_;

  std::string topic_;

  ros::Subscriber map_sub_;

  RosTopicProperty* topic_property_;
  Property*         draw_under_property_;

  multi_map_server::MultiOccupancyGrid::ConstPtr updated_map_;
  multi_map_server::MultiOccupancyGrid::ConstPtr current_map_;
  boost::mutex                                   mutex_;
  bool                                           new_map_;
};

} // namespace rviz

#endif
