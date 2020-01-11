// cheat the compiler

#include <QMouseEvent>
#include <QWheelEvent>

#include "rviz/selection/forwards.h"
#include "rviz/selection/selection_manager.h"

#define private public
#include "rviz/default_plugin/markers/marker_selection_handler.h"
#include "rviz/viewport_mouse_event.h"
#undef private

// #include "gamelikeinput.hpp"

#include "rviz/default_plugin/tools/move_tool.h"
#include "rviz/display_context.h"
#include "rviz/render_panel.h"
#include "rviz/tool_manager.h"

#include "boost/unordered_map.hpp"

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include "rviz/geometry.h"
#include "rviz/load_resource.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/render_panel.h"
#include "rviz/viewport_mouse_event.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include "rviz/properties/string_property.h"

#include "nav_msgs/Path.h"
// #include "quadrotor_msgs/SwarmCommand.h"
#include "std_msgs/Int32MultiArray.h"

//! @todo rewrite this grabage code

void
GameLikeInput::updateTopic()
{
  pub_pointlist =
    nh_.advertise<nav_msgs::Path>(topic_property_wp_->getStdString(), 1);

  pub_selection = nh_.advertise<std_msgs::Int32MultiArray>(
    topic_property_drone_->getStdString(), 1);

  pub_swarm = nh_.advertise<quadrotor_msgs::SwarmCommand>(
    topic_property_swarm_->getStdString(), 1);

  z_max = property_z_max->getFloat();
  z_min = property_z_min->getFloat();
  y_max = property_y_max->getFloat();
  y_min = property_y_min->getFloat();
  x_max = property_x_max->getFloat();
  x_min = property_x_min->getFloat();
}

GameLikeInput::GameLikeInput()
  : rviz::SelectionTool()
  , move_tool_(new rviz::InteractionTool())
  , selecting_(false)
  , sel_start_x_(0)
  , sel_start_y_(0)
  , moving_(false)
{
  shortcut_key_    = 'z';
  access_all_keys_ = true;

  topic_property_wp_ =
    new rviz::StringProperty("TopicPoint", "point_list",
                             "The topic on which to publish navigation goals.",
                             getPropertyContainer(), SLOT(updateTopic()), this);

  topic_property_drone_ =
    new rviz::StringProperty("TopicSelect", "select_list",
                             "The topic on which to publish select drone id.",
                             getPropertyContainer(), SLOT(updateTopic()), this);

  topic_property_swarm_ = new rviz::StringProperty(
    "TopicSwarm", "swarm", "The topic on which to publish swarm command.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  property_z_max = new rviz::FloatProperty(
    "RangeZ_max", 1.7, "", getPropertyContainer(), SLOT(updateTopic()), this);
  property_z_min = new rviz::FloatProperty(
    "RangeZ_min", 0.8, "", getPropertyContainer(), SLOT(updateTopic()), this);

  property_y_max = new rviz::FloatProperty(
    "RangeY_max", 2.5, "", getPropertyContainer(), SLOT(updateTopic()), this);
  property_y_min = new rviz::FloatProperty(
    "RangeY_min", -2.5, "", getPropertyContainer(), SLOT(updateTopic()), this);

  property_x_max = new rviz::FloatProperty(
    "RangeX_max", 4, "", getPropertyContainer(), SLOT(updateTopic()), this);
  property_x_min = new rviz::FloatProperty(
    "RangeX_min", -4, "", getPropertyContainer(), SLOT(updateTopic()), this);
}

GameLikeInput::~GameLikeInput()
{
  delete move_tool_;
  delete topic_property_wp_;
  delete topic_property_drone_;
  delete topic_property_swarm_;

  delete property_z_max;
  delete property_z_min;
  delete property_x_max;
  delete property_x_min;
  delete property_y_max;
  delete property_y_min;
}

void
GameLikeInput::onInitialize()
{
  this->move_tool_->initialize(context_);

  arrow_ = new rviz::Arrow(scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.3f, 0.35f, 0.9f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);

  state_ = None;
  updateTopic();
}

void
GameLikeInput::sendMessage()
{
  std_msgs::Int32MultiArray array;
  nav_msgs::Path            path;

  array.data.clear();

  rviz::SelectionManager* sel_manager = context_->getSelectionManager();

  auto size = std::distance(selection_.begin(), selection_.end());

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;

  //! @note makeup selection array
  if (!selection_.empty())
  {
    for (auto it = selection_.begin(); it != selection_.end(); it++)
    {
      rviz::Picked                  p  = it->second;
      rviz::SelectionHandler*       sh = sel_manager->getHandler(p.handle);
      rviz::MarkerSelectionHandler* mh =
        reinterpret_cast<rviz::MarkerSelectionHandler*>(sh);

      QString cpy = mh->marker_id_;

      //! @note use the mean position of every selected drone as start point
      pose.pose.position.x += mh->getPosition().x;
      pose.pose.position.y += mh->getPosition().y;
      pose.pose.position.z += mh->getPosition().z;

      array.data.push_back(cpy.remove("drone/").toInt());
    }

    pose.pose.position.x /= size;
    pose.pose.position.y /= size;
    pose.pose.position.z /= size;

    path.poses.push_back(pose);
  }

  //! @note go through arrow_array, get the internal parameter of each array
  //! makeup each waypoint

  for (int i = 0; i < arrow_array.size(); ++i)
  {
    rviz::Arrow*               p_a = arrow_array[i];
    geometry_msgs::PoseStamped ps;

    ps.pose.position.x = p_a->getPosition().x;
    ps.pose.position.y = p_a->getPosition().y;
    ps.pose.position.z = z_vector[i];

    if (ps.pose.position.z > z_max)
      ps.pose.position.z = z_max;
    if (ps.pose.position.z < z_min)
      ps.pose.position.z = z_min;

    if (ps.pose.position.y > y_max)
      ps.pose.position.y = y_max;
    if (ps.pose.position.y < y_min)
      ps.pose.position.y = y_min;

    if (ps.pose.position.x > x_max)
      ps.pose.position.x = x_max;
    if (ps.pose.position.x < x_min)
      ps.pose.position.x = x_min;

    path.poses.push_back(ps);

    delete p_a;
  }
  z_vector.clear();
  arrow_array.clear();
  arrow_->getSceneNode()->setVisible(false);

  path.header.frame_id = "map";
  path.header.stamp    = ros::Time::now();

  std::sort(array.data.begin(), array.data.end());

  quadrotor_msgs::SwarmCommand swarm;
  swarm.plan      = path;
  swarm.selection = array.data;

  pub_selection.publish(array);
  pub_pointlist.publish(path);
  pub_swarm.publish(swarm);
}

void
GameLikeInput::onPoseSet(double x, double y, double z, double theta)
{
  ROS_WARN("3D Goal Set");
  std::string    fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(
    tf::Pose(quat, tf::Point(x, y, z)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), "
           "Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n",
           fixed_frame.c_str(), goal.pose.position.x, goal.pose.position.y,
           goal.pose.position.z, goal.pose.orientation.x,
           goal.pose.orientation.y, goal.pose.orientation.z,
           goal.pose.orientation.w, theta);
}

int
GameLikeInput::processMouseEvent(rviz::ViewportMouseEvent& event)
{

  rviz::SelectionManager* sel_manager = context_->getSelectionManager();

  int flags = 0;

  // wp vars
  static double    initz;
  static double    prevz;
  static double    prevangle;
  const double     z_scale = 50;
  Ogre::Quaternion orient_x =
    Ogre::Quaternion(Ogre::Radian(-Ogre::Math::PI), Ogre::Vector3::UNIT_X);

  if (event.alt())
  {
    moving_    = true;
    selecting_ = false;
  }
  else
  {
    moving_ = false;

    if (event.leftDown())
    {
      this->selecting_ = true;

      this->sel_start_x_ = event.x;
      this->sel_start_y_ = event.y;
    }
  }

  if (selecting_)
  {
    rviz::M_Picked selection;

    sel_manager->highlight(event.viewport, sel_start_x_, sel_start_y_, event.x,
                           event.y);

    if (event.leftUp())
    {
      rviz::SelectionManager::SelectType type = rviz::SelectionManager::Replace;

      if (event.shift())
      {
        type = rviz::SelectionManager::Add;
      }
      else if (event.control())
      {
        type = rviz::SelectionManager::Remove;
      }

      //      boost::recursive_mutex::scoped_lock lock(global_mutex_);
      sel_manager->select(event.viewport, this->sel_start_x_,
                          this->sel_start_y_, event.x, event.y, type);
      selection = sel_manager->getSelection();

      rviz::MarkerSelectionHandler* cmp = new rviz::MarkerSelectionHandler(
        nullptr, std::pair<std::string, int32_t>(), context_);

      for (auto it = selection.begin(); it != selection.end();)
      {
        rviz::Picked            p  = it->second;
        rviz::SelectionHandler* sh = sel_manager->getHandler(p.handle);
        void (rviz::SelectionHandler::*vptr)(const rviz::Picked& obj,
                                             rviz::Property* parent_property) =
          &rviz::SelectionHandler::createProperties;
        // black magic, must have warning
        bool e = false;
        if (reinterpret_cast<void*>(sh->*vptr) !=
            reinterpret_cast<void*>(cmp->*vptr))
          e = true;
        else
        {
          rviz::MarkerSelectionHandler* mh =
            reinterpret_cast<rviz::MarkerSelectionHandler*>(sh);
          if (mh->marker_id_.startsWith("drone"))
            e = false;
          else
            e = true;
        }

        if (e)
          it = selection.erase(it);
        else
          it++;
      }
      delete cmp;

      sel_manager->setSelection(selection);
      selection_ = selection;

      selecting_ = false;
    }

    flags |= Render;
  }
  else if (moving_)
  {
    sel_manager->removeHighlight();
    flags = move_tool_->processMouseEvent(event);

    if (event.type == QEvent::MouseButtonRelease)
    {
      moving_ = false;
    }
  }
  else
  {

    if (event.rightDown())
    {
      if (state_ == None)
      {
        Ogre::Vector3 intersection;
        Ogre::Plane   ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
        if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane,
                                              event.x, event.y, intersection))
        {
          pos_ = intersection;
          arrow_->setPosition(pos_);
          arrow_->setOrientation(
            Ogre::Quaternion(Ogre::Radian(0.0), Ogre::Vector3::UNIT_Z) *
            orient_x);
          arrow_->getSceneNode()->setVisible(true);
          state_ = Height;
          flags |= Render;
        }
        initz = pos_.z;
        prevz = event.y;
      }
    }
    else if (event.type == QEvent::MouseMove && event.right())
    {
      if (state_ == Height)
      {
        double z  = event.y;
        double dz = z - prevz;
        prevz     = z;
        pos_.z -= dz / z_scale;
        arrow_->set(pos_.z, 0.08, 0, 0);
        flags |= Render;
      }
    }
    if (event.rightUp())
    {
      onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);

      state_ = None;
      if (event.shift())
      {
        arrow_array.push_back(arrow_);
        z_vector.push_back(pos_.z);
        arrow_ = new rviz::Arrow(scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f);
        arrow_->setColor(0.3f, 0.35f, 0.9f, 1.0f);
        arrow_->getSceneNode()->setVisible(false);
      }
      else
      {
        sendMessage();
      }
      flags |= Render;
    }

    if (!event.shift())
    {
      if (state_ == None && arrow_array.size() != 0)
      {
        sendMessage();
      }
    }

    sel_manager->highlight(event.viewport, event.x, event.y, event.x, event.y);
  }

  return flags;
}

int
GameLikeInput::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  return SelectionTool::processKeyEvent(event, panel);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(GameLikeInput, rviz::Tool)
