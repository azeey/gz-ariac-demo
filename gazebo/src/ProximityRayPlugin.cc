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

#include "ProximityRayPlugin.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <regex>
#include <sdf/Lidar.hh>

using namespace gz;
using namespace sim;

// Register this plugin with the simulator
GZ_ADD_PLUGIN(ProximityRayPlugin,
              sim::System,
              sim::ISystemConfigure,
              sim::ISystemPreUpdate,
              sim::ISystemPostUpdate)


//////////////////////////////////////////////////
std::string ProximityRayPlugin::Topic(std::string topicName) const
{
  std::string globalTopicName = "/";
  // this->GetHandle() is not supported. The "name" attribute of a plugin is
  // used to specify the class name within the plugin.
  globalTopicName += this->parentSensorName + "/" + topicName;
  return std::regex_replace(globalTopicName, std::regex("::"), "/");
}

/////////////////////////////////////////////////
// Analog to void ProximityRayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
void ProximityRayPlugin::Configure(
    const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &_eventMgr)
{
  // Get the name of the parent sensor
  this->parentSensorEntity = _entity;
  this->parentSensorName = sim::removeParentScope(
      sim::scopedName(_entity, _ecm, "::", false), "::");

  if (!_ecm.Component<components::GpuLidar>(_entity))
  {
    gzerr << "ProximityRayPlugin requires a Ray Sensor as its parent";
    return;
  }


  if (_sdf->HasElement("time_delay"))
  {
    // Setting update_rate of sensor is Not supported in Gazebo-Sim
  }

  if (_sdf->HasElement("output_state_topic"))
  {
    this->stateTopic = _sdf->Get<std::string>("output_state_topic");
  }
  else {
    this->stateTopic = this->Topic("sensor_state");
  }

  transport::AdvertiseMessageOptions advertOpts;
  advertOpts.SetMsgsPerSec(50);
  this->statePub =
      this->node.Advertise<msgs::Boolean>(this->stateTopic, advertOpts);

  if (_sdf->HasElement("output_change_topic"))
  {
    this->stateChangeTopic = _sdf->Get<std::string>("output_change_topic");
  }
  else {
    this->stateChangeTopic = this->Topic("state_change");
  }

  this->stateChangePub =
      this->node.Advertise<msgs::Boolean>(this->stateChangeTopic, advertOpts);

  // TODO: override sensor's range values
  /*
  if (_sdf->HasElement("sensing_range_min"))
  {
    this->sensingRangeMin = _sdf->Get<double>("sensing_range_min");
  }
  else {
    this->sensingRangeMin = this->parentSensor->RangeMin();
  }
  gzdbg << "Using mininimum sensing range of: " << this->sensingRangeMin << " m\n";

  if (_sdf->HasElement("sensing_range_max"))
  {
    this->sensingRangeMax = _sdf->Get<double>("sensing_range_max");
  }
  else {
    this->sensingRangeMax = this->parentSensor->RangeMax();
  }
  gzdbg << "Using maximum sensing range of: " << this->sensingRangeMax << " m\n";
  */

  this->useLinkFrame = true;
  if (_sdf->HasElement("use_link_frame"))
  {
    this->useLinkFrame = _sdf->Get<bool>("use_link_frame");
  }
  if (this->useLinkFrame)
  {
    this->linkEntity = *_ecm.ComponentData<components::ParentEntity>(
        this->parentSensorEntity);
  }

  this->objectDetected = false;
  this->validConfig = true;
}

void ProximityRayPlugin::PreUpdate(const sim::UpdateInfo &_info,
                                   sim::EntityComponentManager &_ecm)
{
  static bool ranOnce{false};
  if (!ranOnce && this->validConfig)
  {
    ranOnce = true;
    // TODO(azeey) This currently doesn't work for rendering sensors.
    // See https://github.com/gazebosim/gz-sim/issues/85
    //
    // std::string rayTopicName =
    //     *_ecm.ComponentData<components::SensorTopic>(
    //         this->parentSensorEntity);
    //
    // A copy of the logic in `src/rendering/RenderUtils.cc` to determine the
    // topic of a rendering sensor.
    //
    auto sensorComp =
        _ecm.ComponentData<components::GpuLidar>(this->parentSensorEntity);
    std::string rayTopicName = sensorComp->Topic();
    // check topic
    if (rayTopicName.empty())
    {
      rayTopicName =
          sim::scopedName(this->parentSensorEntity, _ecm) + "/scan";
    }

    gzdbg << "ProximityRayPlugin subscribing to: " << rayTopicName << std::endl;

    this->node.Subscribe(rayTopicName, &ProximityRayPlugin::OnNewLaserScans,
                         this);
  }
}

// Most of the logic in the old ProximityRayPlugin::OnNewLaserScans()
void ProximityRayPlugin::PostUpdate(const sim::UpdateInfo &_info,
                                    const sim::EntityComponentManager &_ecm)
{
  if (!this->validConfig || _info.paused)
    return;

  bool stateChanged = this->ProcessScan(_info, _ecm);


  // Fill message
  this->stateMsg.mutable_header()->mutable_stamp()->CopyFrom(
      sim::convert<msgs::Time>(_info.simTime));
  this->stateMsg.set_data(this->objectDetected);

  // Publish sensor state message
    if (this->statePub && this->statePub->HasConnections()) {
    this->statePub->Publish(this->stateMsg);
  }

  if (stateChanged)
  {
    // Publish state state change message
    if (this->stateChangePub && this->stateChangePub->HasConnections()) {
      this->stateChangePub->Publish(this->stateMsg);
    }
  }
}
/////////////////////////////////////////////////
void ProximityRayPlugin::OnNewLaserScans(const msgs::LaserScan &_msg)
{
  std::lock_guard<std::mutex> lk(this->mutex);
  // Store message and process it in PostUpdate
  this->lastMsg = _msg;
}

/////////////////////////////////////////////////
bool ProximityRayPlugin::ProcessScan(const sim::UpdateInfo &_info,
                                     const sim::EntityComponentManager &_ecm)
{
  bool stateChanged = false;
  // Prevent new scans from arriving while we're processing this one
  // this->parentSensor->SetActive(false); // Not supported
  std::lock_guard<std::mutex>lk (mutex);

  if (!this->lastMsg.has_value())
    return stateChanged;

  auto sensorComp =
      _ecm.ComponentData<components::GpuLidar>(this->parentSensorEntity);
  const auto *ray = sensorComp->LidarSensor();
  this->sensingRangeMax = ray->RangeMax();
  this->sensingRangeMin = ray->RangeMin();

  const auto& ranges = this->lastMsg->ranges();

  bool objectDetected = false;

  for (unsigned int i = 0; i<ranges.size(); i++){
    double range = ranges[i];
    // TODO: determine detections in cylindrical shape not spherical
    if (range < this->sensingRangeMax and range > this->sensingRangeMin) {
      objectDetected = true;
      break;
    }
  }

  if (objectDetected) {
      if (!this->objectDetected) {
      gzdbg << "Object detected\n";
      stateChanged = true;
    }
    this->objectDetected = true;
  }
  else
  {
    if (this->objectDetected) {
      gzdbg << "Object no longer detected\n";
      stateChanged = true;
    }
    this->objectDetected = false;
  }

  if (this->useLinkFrame)
  {
    // TODO: deal with sensors oriented differently
    auto sensorPose = sim::worldPose(this->parentSensorEntity, _ecm);
    this->sensingRangeMin += sensorPose.Pos().X();
    this->sensingRangeMax += sensorPose.Pos().X();
  }

  // this->parentSensor->SetActive(true); // Not supported

  this->lastMsg.reset();
  return stateChanged;
}

