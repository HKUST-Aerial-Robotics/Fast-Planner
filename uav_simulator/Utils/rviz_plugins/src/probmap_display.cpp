/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <boost/bind.hpp>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"

#include "probmap_display.h"

namespace rviz
{

ProbMapDisplay::ProbMapDisplay()
  : Display()
  , manual_object_(NULL)
  //! @bug cannot compile @gcc-5 or later, material_(0)
  , loaded_(false)
  , resolution_(0.0f)
  , width_(0)
  , height_(0)
  , position_(Ogre::Vector3::ZERO)
  , orientation_(Ogre::Quaternion::IDENTITY)
  , new_map_(false)
{
  topic_property_ = new RosTopicProperty(
    "Topic", "", QString::fromStdString(
                   ros::message_traits::datatype<nav_msgs::OccupancyGrid>()),
    "nav_msgs::OccupancyGrid topic to subscribe to.", this,
    SLOT(updateTopic()));

  alpha_property_ = new FloatProperty(
    "Alpha", 0.7, "Amount of transparency to apply to the map.", this,
    SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  draw_under_property_ =
    new Property("Draw Behind", false,
                 "Rendering option, controls whether or not the map is always"
                 " drawn behind everything else.",
                 this, SLOT(updateDrawUnder()));

  resolution_property_ = new FloatProperty(
    "Resolution", 0, "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);

  width_property_ = new IntProperty(
    "Width", 0, "Width of the map, in meters. (not editable)", this);
  width_property_->setReadOnly(true);

  height_property_ = new IntProperty(
    "Height", 0, "Height of the map, in meters. (not editable)", this);
  height_property_->setReadOnly(true);

  position_property_ = new VectorProperty(
    "Position", Ogre::Vector3::ZERO,
    "Position of the bottom left corner of the map, in meters. (not editable)",
    this);
  position_property_->setReadOnly(true);

  orientation_property_ =
    new QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY,
                           "Orientation of the map. (not editable)", this);
  orientation_property_->setReadOnly(true);
}

ProbMapDisplay::~ProbMapDisplay()
{
  unsubscribe();
  clear();
}

void
ProbMapDisplay::onInitialize()
{
  static int        count = 0;
  std::stringstream ss;
  ss << "MapObjectMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create(
    ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);

  updateAlpha();
}

void
ProbMapDisplay::onEnable()
{
  subscribe();
}

void
ProbMapDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void
ProbMapDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  if (!topic_property_->getTopic().isEmpty())
  {
    try
    {
      map_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1,
                                      &ProbMapDisplay::incomingMap, this);
      setStatus(StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Topic",
                QString("Error subscribing: ") + e.what());
    }
  }
}

void
ProbMapDisplay::unsubscribe()
{
  map_sub_.shutdown();
}

void
ProbMapDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();

  Ogre::Pass*             pass     = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                              Ogre::LBS_CURRENT, alpha);

  if (alpha < 0.9998)
  {
    material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->setDepthWriteEnabled(false);
  }
  else
  {
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(!draw_under_property_->getValue().toBool());
  }
}

void
ProbMapDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getValue().toBool();

  if (alpha_property_->getFloat() >= 0.9998)
  {
    material_->setDepthWriteEnabled(!draw_under);
  }

  if (manual_object_)
  {
    if (draw_under)
    {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
    }
    else
    {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
    }
  }
}

void
ProbMapDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  clear();
}

void
ProbMapDisplay::clear()
{
  setStatus(StatusProperty::Warn, "Message", "No map received");

  if (!loaded_)
  {
    return;
  }

  scene_manager_->destroyManualObject(manual_object_);
  manual_object_ = NULL;

  std::string tex_name = texture_->getName();
  texture_.setNull();
  Ogre::TextureManager::getSingleton().unload(tex_name);

  loaded_ = false;
}

bool
validateFloats(const nav_msgs::OccupancyGrid& msg)
{
  bool valid = true;
  valid      = valid && validateFloats(msg.info.resolution);
  valid      = valid && validateFloats(msg.info.origin);
  return valid;
}

void
ProbMapDisplay::update(float wall_dt, float ros_dt)
{
  {
    boost::mutex::scoped_lock lock(mutex_);

    current_map_ = updated_map_;
  }

  if (!current_map_ || !new_map_)
  {
    return;
  }

  if (current_map_->data.empty())
  {
    return;
  }

  new_map_ = false;

  if (!validateFloats(*current_map_))
  {
    setStatus(StatusProperty::Error, "Map",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  if (current_map_->info.width * current_map_->info.height == 0)
  {
    std::stringstream ss;
    ss << "Map is zero-sized (" << current_map_->info.width << "x"
       << current_map_->info.height << ")";
    setStatus(StatusProperty::Error, "Map", QString::fromStdString(ss.str()));
    return;
  }

  clear();

  setStatus(StatusProperty::Ok, "Message", "Map received");

  ROS_DEBUG("Received a %d X %d map @ %.3f m/pix\n", current_map_->info.width,
            current_map_->info.height, current_map_->info.resolution);

  float resolution = current_map_->info.resolution;

  int width  = current_map_->info.width;
  int height = current_map_->info.height;

  Ogre::Vector3 position(current_map_->info.origin.position.x,
                         current_map_->info.origin.position.y,
                         current_map_->info.origin.position.z);
  Ogre::Quaternion orientation(current_map_->info.origin.orientation.w,
                               current_map_->info.origin.orientation.x,
                               current_map_->info.origin.orientation.y,
                               current_map_->info.origin.orientation.z);
  frame_ = current_map_->header.frame_id;
  if (frame_.empty())
  {
    frame_ = "/map";
  }

  // Expand it to be RGB data
  unsigned int   pixels_size = width * height;
  unsigned char* pixels      = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  bool         map_status_set     = false;
  unsigned int num_pixels_to_copy = pixels_size;
  if (pixels_size != current_map_->data.size())
  {
    std::stringstream ss;
    ss << "Data size doesn't match width*height: width = " << width
       << ", height = " << height
       << ", data size = " << current_map_->data.size();
    setStatus(StatusProperty::Error, "Map", QString::fromStdString(ss.str()));
    map_status_set = true;

    // Keep going, but don't read past the end of the data.
    if (current_map_->data.size() < pixels_size)
    {
      num_pixels_to_copy = current_map_->data.size();
    }
  }

  // TODO: a fragment shader could do this on the video card, and
  // would allow a non-grayscale color to mark the out-of-range
  // values.
  for (unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy;
       pixel_index++)
  {
    unsigned char val;
    int8_t        data = current_map_->data[pixel_index];
    if (data > 0)
      val = 0;
    else if (data < 0)
      val = 255;
    else
      val               = 127;
    pixels[pixel_index] = val;
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
  static int        tex_count = 0;
  std::stringstream ss;
  ss << "MapTexture" << tex_count++;
  try
  {
    texture_ = Ogre::TextureManager::getSingleton().loadRawData(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      pixel_stream, width, height, Ogre::PF_L8, Ogre::TEX_TYPE_2D, 0);

    if (!map_status_set)
    {
      setStatus(StatusProperty::Ok, "Map", "Map OK");
    }
  }
  catch (Ogre::RenderingAPIException&)
  {
    Ogre::Image image;
    pixel_stream->seek(0);
    float fwidth  = width;
    float fheight = height;
    if (width > height)
    {
      float aspect = fheight / fwidth;
      fwidth       = 2048;
      fheight      = fwidth * aspect;
    }
    else
    {
      float aspect = fwidth / fheight;
      fheight      = 2048;
      fwidth       = fheight * aspect;
    }

    {
      std::stringstream ss;
      ss
        << "Map is larger than your graphics card supports.  Downsampled from ["
        << width << "x" << height << "] to [" << fwidth << "x" << fheight
        << "]";
      setStatus(StatusProperty::Ok, "Map", QString::fromStdString(ss.str()));
    }

    ROS_WARN("Failed to create full-size map texture, likely because your "
             "graphics card does not support textures of size > 2048.  "
             "Downsampling to [%d x %d]...",
             (int)fwidth, (int)fheight);
    // ROS_INFO("Stream size [%d], width [%f], height [%f], w * h [%f]",
    // pixel_stream->size(), width, height, width * height);
    image.loadRawData(pixel_stream, width, height, Ogre::PF_L8);
    image.resize(fwidth, fheight, Ogre::Image::FILTER_NEAREST);
    ss << "Downsampled";
    texture_ = Ogre::TextureManager::getSingleton().loadImage(
      ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
  }

  delete[] pixels;

  Ogre::Pass*             pass     = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

  static int        map_count = 0;
  std::stringstream ss2;
  ss2 << "MapObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject(ss2.str());
  scene_node_->attachObject(manual_object_);

  manual_object_->begin(material_->getName(),
                        Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(resolution * width, resolution * height, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top left
      manual_object_->position(0.0f, resolution * height, 0.0f);
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Bottom right
      manual_object_->position(resolution * width, 0.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(resolution * width, resolution * height, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }
  }
  manual_object_->end();

  if (draw_under_property_->getValue().toBool())
  {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  resolution_property_->setValue(resolution);
  width_property_->setValue(width);
  height_property_->setValue(height);
  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);

  transformMap();

  loaded_ = true;

  context_->queueRender();
}

void
ProbMapDisplay::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{

  updated_map_ = msg;
  boost::mutex::scoped_lock lock(mutex_);
  new_map_ = true;
}

void
ProbMapDisplay::transformMap()
{
  if (!current_map_)
  {
    return;
  }

  Ogre::Vector3    position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(
        frame_, ros::Time(), current_map_->info.origin, position, orientation))
  {
    ROS_DEBUG("Error transforming map '%s' from frame '%s' to frame '%s'",
              qPrintable(getName()), frame_.c_str(), qPrintable(fixed_frame_));

    setStatus(StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame_) +
                "] to [" + fixed_frame_ + "]");
  }
  else
  {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void
ProbMapDisplay::fixedFrameChanged()
{
  transformMap();
}

void
ProbMapDisplay::reset()
{
  Display::reset();

  clear();
  // Force resubscription so that the map will be re-sent
  updateTopic();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::ProbMapDisplay, rviz::Display)
