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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/laserscan.pb.h>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <optional>

namespace sim = gz::sim;
namespace transport = gz::transport;
namespace msgs = gz::msgs;

/// \brief A Ray Sensor Plugin which makes it act as a proximity sensor
class ProximityRayPlugin 
      : public sim::System,
        public sim::ISystemConfigure,
        public sim::ISystemPreUpdate,
        public sim::ISystemPostUpdate
  {
    /// \brief Update callback
    public: void OnNewLaserScans(const msgs::LaserScan &_msg);

    /// \brief Process the scan data and update state
    /// \returns true if the state has changed since processing the last scan
    public: bool ProcessScan(const sim::UpdateInfo &_info,
                             const sim::EntityComponentManager &_ecm);

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Configure(const sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           sim::EntityComponentManager &_ecm,
                           sim::EventManager &_eventMgr);

    public: void PreUpdate(
                const sim::UpdateInfo &_info,
                sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(
                const sim::UpdateInfo &_info,
                const sim::EntityComponentManager &_ecm) override;
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
    protected: sim::Entity linkEntity{sim::kNullEntity};

    /// \brief Pointer to world
    protected: sim::Entity worldEntity{sim::kNullEntity};

    /// \brief Pointer to this node for publishing
    protected: transport::Node node;

    /// \brief The parent sensor
    protected: sim::Entity parentSensorEntity{sim::kNullEntity};
    protected: std::string parentSensorName;

    protected: bool validConfig{false};

  };

#endif /* end of include guard: PROXIMITYRAYPLUGIN_HH_ */
