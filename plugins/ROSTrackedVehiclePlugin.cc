/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

//#include <ignition/math/Vector3.hh>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/ROSTrackedVehiclePlugin.hh"

using namespace gazebo;

namespace gazebo
{
bool trackedVehiclePoseWarningIssued = false;
}

/// \brief Private data class
class gazebo::ROSTrackedVehiclePluginPrivate
{
/// \brief Pointer to model containing plugin.
public: physics::ModelPtr model;

/// \brief SDF for this plugin;
public: sdf::ElementPtr sdf;

/// \brief Pointer to a node with robot prefix.
public: transport::NodePtr robotNode;

/// \brief Velocity command subscriber.
public: transport::SubscriberPtr velocityPoseSub;

/// \brief Distance between the centers of the tracks.
public: double tracksSeparation = 0.1;

/// \brief Steering efficiency coefficient (between 0.0 and 1.0).
public: double steeringEfficiency = 0.5;

/// \brief Max linear velocity in m/s. Also max track velocity.
public: double maxLinearSpeed = 1.0;

/// \brief Max angular speed in rad/s.
public: double maxAngularSpeed = 1.0;

/// \brief Friction coefficient in the first friction direction.
public: boost::optional<double> trackMu;

/// \brief Friction coefficient in the second friction direction.
public: boost::optional<double> trackMu2;

/// \brief Namespace used as a prefix for gazebo topic names.
public: std::string robotNamespace;

//
// ROS Communication
//

/// \brief Velocity command subscriber.
public: ros::Subscriber velocityTwistSub;
public: ros::Subscriber rightTrackSub;
public: ros::Subscriber leftTrackSub;

/// \brief Publisher of the track velocities.
public: ros::Publisher tracksVelocityPub;

public: std::string cmdVelTopic = "cmd_vel";
public: bool listenToCmdVel = true;

/// \brief Track topics
public: std::string leftTrack;
public: std::string rightTrack;

/// \brief Used for ros communication
public: std::unique_ptr<ros::NodeHandle> rosNode;
};

ROSTrackedVehiclePlugin::ROSTrackedVehiclePlugin()
        : dataPtr(new ROSTrackedVehiclePluginPrivate)
{
        this->trackNames[Tracks::LEFT] = "left";
        this->trackNames[Tracks::RIGHT] = "right";
}

ROSTrackedVehiclePlugin::~ROSTrackedVehiclePlugin() = default;

void ROSTrackedVehiclePlugin::Load(physics::ModelPtr _model,
                                   sdf::ElementPtr _sdf)
{
        GZ_ASSERT(_model, "ROSTrackedVehiclePlugin _model pointer is NULL");
        this->dataPtr->model = _model;

        GZ_ASSERT(_sdf, "ROSTrackedVehiclePlugin _sdf pointer is NULL");
        this->dataPtr->sdf = _sdf;

        // Load parameters from SDF plugin contents.
        this->LoadParam(_sdf, "robot_namespace", this->dataPtr->robotNamespace,
                        _model->GetName());
        this->LoadParam(_sdf, "steering_efficiency",
                        this->dataPtr->steeringEfficiency, 0.5);
        this->LoadParam(_sdf, "tracks_separation",
                        this->dataPtr->tracksSeparation, 0.4);
        this->LoadParam(_sdf, "max_linear_speed",
                        this->dataPtr->maxLinearSpeed, 1.);
        this->LoadParam(_sdf, "max_angular_speed",
                        this->dataPtr->maxAngularSpeed, 1.);

        if(_sdf->HasElement("left_track_topic") && _sdf->HasElement("right_track_topic")) {
                this->LoadParam(_sdf, "left_track_topic",
                                this->dataPtr->leftTrack, "/leftTrack");
                this->LoadParam(_sdf, "right_track_topic",
                                this->dataPtr->rightTrack, "/rightTrack");
                this->dataPtr->listenToCmdVel = false;
        }

        // Get Cmd Vel Topic and validate
        if(this->dataPtr->listenToCmdVel) {
                if (_sdf->HasElement("cmd_vel_topic")) {
                        this->LoadParam(_sdf, "cmd_vel_topic", this->dataPtr->cmdVelTopic, "/"+this->dataPtr->robotNamespace+"/cmd_vel");
                }
                if(this->dataPtr->cmdVelTopic[0] != '/'){
                  this->dataPtr->cmdVelTopic = "/" + this->dataPtr->cmdVelTopic;
                }
        }


        if (_sdf->HasElement("track_mu"))
        {
                double mu;
                this->LoadParam(_sdf, "track_mu", mu, 2.0);
                this->dataPtr->trackMu = mu;
        }

        if (_sdf->HasElement("track_mu2"))
        {
                double mu2;
                this->LoadParam(_sdf, "track_mu2", mu2, 0.5);
                this->dataPtr->trackMu2 = mu2;
        }

        if (this->dataPtr->steeringEfficiency <= 0.)
                throw std::runtime_error("Steering efficiency must be positive");
        if (this->dataPtr->tracksSeparation <= 0.)
                throw std::runtime_error("Tracks separation must be positive");
        if (this->dataPtr->maxLinearSpeed <= 0.)
                throw std::runtime_error("Maximum linear speed must be positive");
        if (this->dataPtr->maxAngularSpeed < 0.)
                throw std::runtime_error("Maximum angular speed must be non-negative");
}

void ROSTrackedVehiclePlugin::Init()
{
        // Initialize ROS Node if it has not already been initialized
        if (!ros::isInitialized())
        {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                          ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->dataPtr->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // Initialize transport nodes.

        // Prepend world name to robot namespace if it isn't absolute.
        auto robotNamespace = this->GetRobotNamespace();
        if (!robotNamespace.empty() && robotNamespace.at(0) != '/')
        {
                robotNamespace = this->dataPtr->model->GetWorld()->Name() +
                                 "/" + robotNamespace;
        }
        this->dataPtr->robotNode = transport::NodePtr(new transport::Node());
        this->dataPtr->robotNode->Init(robotNamespace);

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

        this->dataPtr->velocityPoseSub =
                this->dataPtr->robotNode->Subscribe<msgs::Pose, ROSTrackedVehiclePlugin>(
                        "~/cmd_vel", &ROSTrackedVehiclePlugin::OnVelMsg, this);

#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

        // Spin up the queue helper thread.
        this->rosQueueThread =
                std::thread(std::bind(&ROSTrackedVehiclePlugin::QueueThread, this));

        this->dataPtr->tracksVelocityPub =
                this->dataPtr->rosNode->advertise<geometry_msgs::Vector3>("tracks_speed", 1000);

        if(this->dataPtr->listenToCmdVel) {
                gzmsg << "Subscribing to cmd_vel_topic: " << this->dataPtr->cmdVelTopic << std::endl;
                this->dataPtr->velocityTwistSub =
                        this->dataPtr->rosNode->subscribe(this->dataPtr->cmdVelTopic,
                                                          1000, &ROSTrackedVehiclePlugin::OnRosMsg, this);
        } else {
                gzmsg << "Susbscribing to tracks: Left Track: " << this->dataPtr->leftTrack << ", Right Track: " << this->dataPtr->rightTrack << std::endl;
                this->dataPtr->leftTrackSub =
                        this->dataPtr->rosNode->subscribe(this->dataPtr->leftTrack,
                                                          1000,&ROSTrackedVehiclePlugin::OnLeftTrackMsg, this);
                this->dataPtr->rightTrackSub =
                        this->dataPtr->rosNode->subscribe(this->dataPtr->rightTrack,
                                                          1000,&ROSTrackedVehiclePlugin::OnRightTrackMsg, this);
        }
}

void ROSTrackedVehiclePlugin::Reset()
{
        this->SetTrackVelocity(0., 0.);

        ModelPlugin::Reset();
}

void ROSTrackedVehiclePlugin::SetTrackVelocity(double _left, double _right)
{
        // Apply the max track velocity limit.

        const auto left = ignition::math::clamp(_left,
                                                -this->dataPtr->maxLinearSpeed,
                                                this->dataPtr->maxLinearSpeed);
        const auto right = ignition::math::clamp(_right,
                                                 -this->dataPtr->maxLinearSpeed,
                                                 this->dataPtr->maxLinearSpeed);

        // Call the descendant custom handler of the subclass.
        this->SetTrackVelocityImpl(left, right);

        // Publish the resulting track velocities to anyone who is interested.
        auto speedMsg = geometry_msgs::Vector3();
        speedMsg.x = left;
        speedMsg.y = right;
        speedMsg.z = 0;
        this->dataPtr->tracksVelocityPub.publish(speedMsg);
}

void ROSTrackedVehiclePlugin::SetBodyVelocity(
        const double _linear, const double _angular)
{
        std::lock_guard<std::mutex> lock(this->mutex);

        // Compute effective linear and angular speed.
        const auto linearSpeed = ignition::math::clamp(
                _linear,
                -this->dataPtr->maxLinearSpeed,
                this->dataPtr->maxLinearSpeed);

        const auto angularSpeed = ignition::math::clamp(
                _angular,
                -this->dataPtr->maxAngularSpeed,
                this->dataPtr->maxAngularSpeed);

        // Compute track velocities using the tracked vehicle kinematics model.
        const auto leftVelocity = linearSpeed + angularSpeed *
                                  this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

        const auto rightVelocity = linearSpeed - angularSpeed *
                                   this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

        // Call the track velocity handler (which does the actual vehicle control).
        this->SetTrackVelocity(leftVelocity, rightVelocity);
}

void ROSTrackedVehiclePlugin::OnVelMsg(ConstPosePtr &_msg)
{
        if (!trackedVehiclePoseWarningIssued)
        {
                gzwarn << "Controlling tracked vehicles via Pose messages is deprecated. "
                        "Use the Twist API via ~/cmd_vel_twist." << std::endl;
                trackedVehiclePoseWarningIssued = true;
        }
        const auto yaw = msgs::ConvertIgn(_msg->orientation()).Euler().Z();
        this->SetBodyVelocity(_msg->position().x(), yaw);
}

void ROSTrackedVehiclePlugin::OnVelMsg(ConstTwistPtr &_msg)
{
        this->SetBodyVelocity(_msg->linear().x(), _msg->angular().z());
}

std::string ROSTrackedVehiclePlugin::GetRobotNamespace()
{
        return this->dataPtr->robotNamespace;
}

double ROSTrackedVehiclePlugin::GetSteeringEfficiency()
{
        return this->dataPtr->steeringEfficiency;
}

void ROSTrackedVehiclePlugin::SetSteeringEfficiency(double _steeringEfficiency)
{
        this->dataPtr->steeringEfficiency = _steeringEfficiency;
        this->dataPtr->sdf->GetElement("steering_efficiency")
        ->Set(_steeringEfficiency);
}

double ROSTrackedVehiclePlugin::GetTracksSeparation()
{
        return this->dataPtr->tracksSeparation;
}

boost::optional<double> ROSTrackedVehiclePlugin::GetTrackMu()
{
        return this->dataPtr->trackMu;
}

void ROSTrackedVehiclePlugin::SetTrackMu(double _mu)
{
        this->dataPtr->trackMu = _mu;
        this->dataPtr->sdf->GetElement("track_mu")->Set(_mu);
        this->UpdateTrackSurface();
}

boost::optional<double> ROSTrackedVehiclePlugin::GetTrackMu2()
{
        return this->dataPtr->trackMu2;
}

void ROSTrackedVehiclePlugin::SetTrackMu2(double _mu2)
{
        this->dataPtr->trackMu2 = _mu2;
        this->dataPtr->sdf->GetElement("track_mu2")->Set(_mu2);
        this->UpdateTrackSurface();
}

void ROSTrackedVehiclePlugin::SetLinkMu(const physics::LinkPtr &_link)
{
        if (!this->GetTrackMu().is_initialized() &&
            !this->GetTrackMu2().is_initialized())
        {
                return;
        }

        for (auto const &collision : _link->GetCollisions())
        {
                auto frictionPyramid = collision->GetSurface()->FrictionPyramid();
                if (frictionPyramid == nullptr)
                {
                        gzwarn << "This dynamics engine doesn't support setting mu/mu2 friction"
                                " parameters. Use its dedicated friction setting mechanism to set the"
                                " wheel friction." << std::endl;
                        break;
                }


                if (this->GetTrackMu().is_initialized())
                {
                        double mu = this->GetTrackMu().get();
                        if (!ignition::math::equal(frictionPyramid->MuPrimary(), mu, 1e-6))
                        {
                                gzdbg << "Setting mu (friction) of link '" << _link->GetName() <<
                                        "' from " << frictionPyramid->MuPrimary() << " to " <<
                                        mu << std::endl;
                        }
                        frictionPyramid->SetMuPrimary(mu);
                }

                if (this->GetTrackMu2().is_initialized())
                {
                        double mu2 = this->GetTrackMu2().get();
                        if (!ignition::math::equal(frictionPyramid->MuSecondary(), mu2, 1e-6))
                        {
                                gzdbg << "Setting mu2 (friction) of link '" << _link->GetName() <<
                                        "' from " << frictionPyramid->MuSecondary() << " to " <<
                                        mu2 << std::endl;
                        }
                        frictionPyramid->SetMuSecondary(mu2);
                }
        }
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
void ROSTrackedVehiclePlugin::OnRosMsg(const geometry_msgs::TwistStamped& _msg)
{
        this->SetBodyVelocity(_msg.twist.linear.x, _msg.twist.angular.z);
}

void ROSTrackedVehiclePlugin::OnLeftTrackMsg(const std_msgs::Float64& _msg)
{
        this->SetLeftTrackVelocityImpl(_msg);
}
void ROSTrackedVehiclePlugin::OnRightTrackMsg(const std_msgs::Float64& _msg)
{
        this->SetRightTrackVelocityImpl(_msg);
}

/// \brief ROS helper function that processes messages
void ROSTrackedVehiclePlugin::QueueThread()
{
        static const double timeout = 0.01;
        while (this->dataPtr->rosNode->ok())
        {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
}
