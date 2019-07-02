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

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "multi_probmap_display.h"

namespace rviz
{

MultiProbMapDisplay::MultiProbMapDisplay()
  : Display()
  , loaded_( false )
  , new_map_(false)
{
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<multi_map_server::MultiOccupancyGrid>() ),
                                          "multi_map_server::MultiOccupancyGrid topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  draw_under_property_ = new Property( "Draw Behind", false,
                                       "Rendering option, controls whether or not the map is always"
                                       " drawn behind everything else.",
                                       this, SLOT( updateDrawUnder() ));
}

MultiProbMapDisplay::~MultiProbMapDisplay()
{
  unsubscribe();
  clear();
}

void MultiProbMapDisplay::onInitialize()
{
}

void MultiProbMapDisplay::onEnable()
{
  subscribe();
}

void MultiProbMapDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void MultiProbMapDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if( !topic_property_->getTopic().isEmpty() )
  {
    try
    {
      map_sub_ = update_nh_.subscribe( topic_property_->getTopicStd(), 1, &MultiProbMapDisplay::incomingMap, this );
      setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
    }
  }
}

void MultiProbMapDisplay::unsubscribe()
{
  map_sub_.shutdown();
}

void MultiProbMapDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getValue().toBool();

  for (unsigned int k = 0; k < material_.size(); k++)
    material_[k]->setDepthWriteEnabled( !draw_under );

  for (unsigned int k = 0; k < manual_object_.size(); k++)
  {
    if( draw_under )
      manual_object_[k]->setRenderQueueGroup( Ogre::RENDER_QUEUE_4 );
    else
      manual_object_[k]->setRenderQueueGroup( Ogre::RENDER_QUEUE_MAIN );
  }
}

void MultiProbMapDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  clear();
}

void MultiProbMapDisplay::clear()
{
  setStatus( StatusProperty::Warn, "Message", "No map received" );

  if( !loaded_ )
  {
    return;
  }
  
  for (unsigned k = 0; k < manual_object_.size(); k++)
  {
    scene_manager_->destroyManualObject( manual_object_[k] );
    std::string tex_name = texture_[k]->getName();
    texture_[k].setNull();
    Ogre::TextureManager::getSingleton().unload( tex_name );
  }
  manual_object_.clear();
  texture_.clear();
  material_.clear();

  loaded_ = false;
}

// ***********************************************************************************************************************************

void MultiProbMapDisplay::update( float wall_dt, float ros_dt )
{  
  {
    boost::mutex::scoped_lock lock(mutex_);
    current_map_ = updated_map_;
  }

  if (!new_map_)
    return;
  new_map_ = false;  

  clear();
  
  //ros::Time t[5];
  //double dt[4] = {0,0,0,0};
  for (unsigned int k = 0; k < current_map_->maps.size(); k++)
  {
    if (current_map_->maps[k].data.empty())
      continue;    
    setStatus( StatusProperty::Ok, "Message", "Map received" );  
    
    // Map info
    float resolution = current_map_->maps[k].info.resolution;  
    int   width      = current_map_->maps[k].info.width;
    int   height     = current_map_->maps[k].info.height;    
    
    // Load pixel
    //t[0] = ros::Time::now(); 
    unsigned int pixels_size = width * height;
    unsigned char* pixels = new unsigned char[pixels_size];
    memset(pixels, 255, pixels_size);
    unsigned int num_pixels_to_copy = pixels_size;
    if( pixels_size != current_map_->maps[k].data.size() )
      if( current_map_->maps[k].data.size() < pixels_size )
        num_pixels_to_copy = current_map_->maps[k].data.size();
    for( unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++ )
    {
      unsigned char val;
      int8_t data = current_map_->maps[k].data[ pixel_index ];     
      if(data > 0)
        val = 255;
      else if(data < 0)
        val = 180;
      else
        val = 0;                     
      pixels[ pixel_index ] = val;
    }
/*        
    int pixels_size = current_map_->maps[k].data.size();    
    unsigned char* pixels = new unsigned char[pixels_size];
    memcpy(pixels, &current_map_->maps[k].data[0], pixels_size);
*/
    // Set texture
    //t[1] = ros::Time::now();
    Ogre::DataStreamPtr pixel_stream;
    pixel_stream.bind( new Ogre::MemoryDataStream( pixels, pixels_size ));
    static int tex_count = 0;
    std::stringstream ss1;
    ss1 << "MultiMapTexture" << tex_count++;
    Ogre::TexturePtr _texture_;
    //t[2] = ros::Time::now();            
    _texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss1.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                  pixel_stream, width, height, Ogre::PF_L8, Ogre::TEX_TYPE_2D, 0);   
    //t[3] = ros::Time::now();                                                                                                                                                                                   
    texture_.push_back(_texture_);                                                
    delete [] pixels;     
    setStatus( StatusProperty::Ok, "Map", "Map OK" );      
    //t[4] = ros::Time::now();     
    
    // Set material
    static int material_count = 0;
    std::stringstream ss0;
    ss0 << "MultiMapObjectMaterial" << material_count++;  
    Ogre::MaterialPtr _material_;
    _material_ = Ogre::MaterialManager::getSingleton().create( ss0.str(),
                                                               Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    _material_->setReceiveShadows(false);
    _material_->getTechnique(0)->setLightingEnabled(false);
    _material_->setDepthBias( -16.0f, 0.0f );
    _material_->setCullingMode( Ogre::CULL_NONE );
    _material_->setDepthWriteEnabled(false);
    material_.push_back(_material_);
    material_.back()->setSceneBlending( Ogre::SBT_TRANSPARENT_COLOUR );      
    material_.back()->setDepthWriteEnabled( false );    
    Ogre::Pass* pass = material_.back()->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = NULL;
    if (pass->getNumTextureUnitStates() > 0)
      tex_unit = pass->getTextureUnitState(0);
    else
      tex_unit = pass->createTextureUnitState();
    tex_unit->setTextureName(texture_.back()->getName());
    tex_unit->setTextureFiltering( Ogre::TFO_NONE );                                                                             
    
    // Set manual object    
    static int map_count = 0;
    std::stringstream ss2;
    ss2 << "MultiMapObject" << map_count++;
    Ogre::ManualObject* _manual_object_ = scene_manager_->createManualObject( ss2.str() );
    manual_object_.push_back(_manual_object_);
    scene_node_->attachObject( manual_object_.back() );    
    float yo  = tf::getYaw(current_map_->origins[k].orientation);
    float co  = cos(yo);
    float so  = sin(yo);
    float dxo = current_map_->origins[k].position.x;
    float dyo = current_map_->origins[k].position.y;     
    float ym  = tf::getYaw(current_map_->maps[k].info.origin.orientation);
    float dxm = current_map_->maps[k].info.origin.position.x;
    float dym = current_map_->maps[k].info.origin.position.y;  
    float yaw = yo + ym;
    float c   = cos(yaw);
    float s   = sin(yaw);    
    float dx  = co * dxm - so * dym + dxo;
    float dy  = so * dxm + co * dym + dyo;
    float x   = 0.0;
    float y   = 0.0;    
    manual_object_.back()->begin(material_.back()->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    {
      // First triangle
      {
        // Bottom left
        x = c * 0.0 - s * 0.0 + dx;
        y = s * 0.0 + c * 0.0 + dy;
        manual_object_.back()->position( x, y, 0.0f );
        manual_object_.back()->textureCoord(0.0f, 0.0f);
        manual_object_.back()->normal( 0.0f, 0.0f, 1.0f );

        // Top right
        x = c * resolution*width - s * resolution*height + dx;
        y = s * resolution*width + c * resolution*height + dy;      
        manual_object_.back()->position( x, y, 0.0f );
        manual_object_.back()->textureCoord(1.0f, 1.0f);
        manual_object_.back()->normal( 0.0f, 0.0f, 1.0f );

        // Top left
        x = c * 0.0 - s * resolution*height + dx;
        y = s * 0.0 + c * resolution*height + dy;        
        manual_object_.back()->position( x, y, 0.0f );
        manual_object_.back()->textureCoord(0.0f, 1.0f);
        manual_object_.back()->normal( 0.0f, 0.0f, 1.0f );
      }

      // Second triangle
      {
        // Bottom left
        x = c * 0.0 - s * 0.0 + dx;
        y = s * 0.0 + c * 0.0 + dy;
        manual_object_.back()->position( x, y, 0.0f );
        manual_object_.back()->textureCoord(0.0f, 0.0f);
        manual_object_.back()->normal( 0.0f, 0.0f, 1.0f );

        // Bottom right
        x = c * resolution*width - s * 0.0 + dx;
        y = s * resolution*width + c * 0.0 + dy;      
        manual_object_.back()->position( x, y, 0.0f );      
        manual_object_.back()->textureCoord(1.0f, 0.0f);
        manual_object_.back()->normal( 0.0f, 0.0f, 1.0f );

        // Top right
        x = c * resolution*width - s * resolution*height + dx;
        y = s * resolution*width + c * resolution*height + dy;      
        manual_object_.back()->position( x, y, 0.0f );
        manual_object_.back()->textureCoord(1.0f, 1.0f);
        manual_object_.back()->normal( 0.0f, 0.0f, 1.0f );
      }
    }
    manual_object_.back()->end();  
    if( draw_under_property_->getValue().toBool() )
      manual_object_.back()->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);  
      
    //for (int i = 0; i < 4; i++)
    //  dt[i] += (t[i+1] - t[i]).toSec();
  }
  loaded_ = true;  
  context_->queueRender(); 
  //ROS_ERROR("RVIZ MAP:  %f %f %f %f", dt[0],dt[1],dt[2],dt[3]);
}

// ***********************************************************************************************************************************

void MultiProbMapDisplay::incomingMap(const multi_map_server::MultiOccupancyGrid::ConstPtr& msg)
{
  updated_map_ = msg;
  boost::mutex::scoped_lock lock(mutex_);
  new_map_ = true;
}

void MultiProbMapDisplay::reset()
{
  Display::reset();
  clear();
  updateTopic();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::MultiProbMapDisplay, rviz::Display )
