/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _HANDSIM_HAPTIX_WORLD_PLUGIN_HH_
#define _HANDSIM_HAPTIX_WORLD_PLUGIN_HH_

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <sdf/sdf.hh>

#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>

#include <ignition/transport.hh>


/// \brief Class for representing a wrench applied to a link over a duration.
class WrenchDuration
{
  public:
    gazebo::physics::LinkPtr link;
    gazebo::WrenchHelper wrench;
    gazebo::common::Time timeRemaining;
    bool persistent;

    WrenchDuration() : persistent(false) {}
    WrenchDuration(gazebo::physics::LinkPtr _link, gazebo::WrenchHelper _wrench,
        gazebo::common::Time _duration, bool _persistent)
      : link(_link), wrench(_wrench), timeRemaining(_duration),
        persistent(_persistent) {}
};

/// \brief Server for responding to the HAPTIX Sim API calls over Ignition
/// transport.
class HaptixWorldPlugin : public gazebo::WorldPlugin
{
  /// \brief Constructor.
  public: HaptixWorldPlugin();

  /// \brief Destructor.
  public: ~HaptixWorldPlugin();

  /// \brief Load the plugin.
  /// \param[in] _world Pointer to world
  /// \param[in] _sdf Pointer to the SDF configuration.
  public: virtual void Load(gazebo::physics::WorldPtr _world,
      sdf::ElementPtr _sdf);

  /// \brief Reset the plugin.
  public: virtual void Reset();

  ///////////// Utility functions /////////////

  ///////////// Protected helpers  //////////////
  /// \brief Read the initial colors of each model from SDF.
  protected: void InitializeColorMap();

  /// \brief Callback on world update
  private: void OnWorldUpdate();

  ///////////// Member variables /////////////

  /// \brief World pointer.
  protected: gazebo::physics::WorldPtr world;

  /// \brief SDF pointer.
  protected: sdf::ElementPtr sdf;

  /// \brief Vector of all the models in the world
  protected: gazebo::physics::Model_V modelVector;

  /// \brief World update connection for updating modelVector
  protected: gazebo::event::ConnectionPtr worldUpdateConnection;

  /// \brief Node for Gazebo transport.
  protected: gazebo::transport::NodePtr gzNode;

  /// \brief ignition transport node for talking to haptix comm
  private: ignition::transport::Node ignNode;

  /// \brief For publishing commands to the server
  private: gazebo::transport::PublisherPtr worldControlPub;

  /// \brief For publishing pause commands
  private: gazebo::transport::PublisherPtr pausePub;

  /// \brief For publishing visual messages to ~/visual
  private: gazebo::transport::PublisherPtr visPub;

  /// \brief For publishing viewpoint messages to ~/user_camera/pose
  private: gazebo::transport::PublisherPtr userCameraPub;

  /// \brief For subscribing to viewpoint messages on ~/user_camera/pose
  private: gazebo::transport::SubscriberPtr userCameraSub;

  /// \brief For storing forces and torques applied over time.
  private: std::vector<WrenchDuration> wrenchDurations;

  /// \brief Maps model IDs to colors
  private: std::map<int, gazebo::common::Color> lastKnownColors;

  /// \brief The functions to execute in OnWorldUpdate
  private: std::vector<std::function<void()>> updateFunctions;

  /// \brief Last time the effort vectors were updates
  private: gazebo::common::Time lastSimUpdateTime;

  /// \brief Mutex to protect the World pointer
  private: std::mutex worldMutex;

  /// \brief The initial camera pose, from SDF.
  private: gazebo::math::Pose initialCameraPose;

  /// \brief Last known viewpoint pose
  private: gazebo::math::Pose userCameraPose;

  /// \brief True if a user camera pose message has been published.
  private: bool userCameraPoseValid;
};

#endif
