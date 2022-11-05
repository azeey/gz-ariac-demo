/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef PROXIMITYRAYPLUGIN_HH_
#define PROXIMITYRAYPLUGIN_HH_

#include <ignition/msgs/boolean.pb.h>

#include <optional>

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

namespace gazebo = ignition::gazebo;
namespace transport = ignition::transport;
namespace msgs = ignition::msgs;

/// \brief A Ray Sensor Plugin which makes it act as a proximity sensor
class ProximityRayPlugin 
      : public gazebo::System,
        public gazebo::ISystemConfigure,
        public gazebo::ISystemPreUpdate,
        public gazebo::ISystemPostUpdate
  {
    /// \brief Update callback
    public: void OnNewLaserScans(const msgs::LaserScan &_msg);

    /// \brief Process the scan data and update state
    /// \returns true if the state has changed since processing the last scan
    public: bool ProcessScan(const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm);

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Configure(const gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gazebo::EntityComponentManager &_ecm,
                           gazebo::EventManager &_eventMgr);

    public: void PreUpdate(
                const gazebo::UpdateInfo &_info,
                gazebo::EntityComponentManager &_ecm) override;

    public: void PostUpdate(
                const gazebo::UpdateInfo &_info,
                const gazebo::EntityComponentManager &_ecm) override;
    /// \brief Generate a scoped topic name from a local one
    /// \param local local topic name
    protected: std::string Topic(std::string topicName) const;

    /// \brief Publisher for the sensor state
    protected: std::optional<transport::Node::Publisher> statePub;

    /// \brief Publisher for the sensor state change
    protected: std::optional<transport::Node::Publisher> stateChangePub;

    /// \brief State message
    protected: msgs::Boolean stateMsg;
    protected: std::optional<msgs::LaserScan> lastMsg;

    /// \brief Mutex to protect interruptionMsg
    protected: std::mutex mutex;

    /// \brief Topic name for state message
    protected: std::string stateTopic;

    /// \brief Topic name for state change message
    protected: std::string stateChangeTopic;

    /// \brief Sensor detection state
    protected: bool objectDetected;

    /// \brief Convert sensor ranges to parent link frame?
    protected: bool useLinkFrame;

    /// \brief Minimum sensing range in meters
    protected: double sensingRangeMin;

    /// \brief Maximum sensing range in meters
    protected: double sensingRangeMax;

    /// \brief Whether or not the output function is normally open (default) or normally closed
    protected: bool normallyOpen;

    /// \brief Pointer to parent link
    protected: gazebo::Entity linkEntity{gazebo::kNullEntity};

    /// \brief Pointer to world
    protected: gazebo::Entity worldEntity{gazebo::kNullEntity};

    /// \brief Pointer to this node for publishing
    protected: transport::Node node;

    /// \brief The parent sensor
    protected: gazebo::Entity parentSensorEntity{gazebo::kNullEntity};
    protected: std::string parentSensorName;

    protected: bool validConfig{false};

  };

#endif /* end of include guard: PROXIMITYRAYPLUGIN_HH_ */
