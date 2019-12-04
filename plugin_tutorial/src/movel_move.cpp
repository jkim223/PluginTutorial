/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

          // Store the pointer to the model
          this->model = _parent;


          // Listen to the update event. This event is broadcast every simulation iteration.
          this->updateConnection = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&ModelPush::OnUpdate, this, _1));



          // Set the update period
          if (_sdf->HasElement("update_period")){
              this->updatePeriod = _sdf->Get<double>("update_period");
          }


          if(_sdf->HasElement("link")){
              this->link_name = _sdf->Get<std::string>("link");
          }


          if(this->link_name.compare("link1") == 0){
              obstacle_index = 1;
          }
          else if(this->link_name.compare("link2") == 0){
              obstacle_index = 2;
          }
          else if(this->link_name.compare("link3") == 0){
              obstacle_index = 3;
          }
          else if(this->link_name.compare("link4") == 0){
              obstacle_index = 4;
          }
          else if(this->link_name.compare("link5") == 0){
              obstacle_index = 5;
          }
          else if(this->link_name.compare("link6") == 0){
              obstacle_index = 6;
          }


          switch(obstacle_index){
          case 1:
              vel_linear_x = -0.2;  vel_linear_y = 0.2;
              break;
          case 4:
              vel_linear_x = 0.1;  vel_linear_y = -0.1;
              break;
          case 3:
              vel_linear_x = 0.2;   vel_linear_y = 0.0;
              break;
          case 2:
              vel_linear_x = 0.0;   vel_linear_y = 0.2;
              break;
          case 5:
              vel_linear_x = -0.2;   vel_linear_y = -0.2;
              break;
          case 6:
              vel_linear_x = 0.2;   vel_linear_y = 0.2;
              break;
          default:
              break;
          }

    }

      // Called by the world update start event
      public: void OnUpdate(const common::UpdateInfo & _info)
      {


          if(_info.simTime - this->prevUpdate > this->updatePeriod){ //enough time has elapsed

              vel_linear_x *= -1;   vel_linear_y *= -1;     vel_linear_z = 0;

              this->prevUpdate = _info.simTime;
          }

              // Apply a small linear velocity to the model.
              this->model->SetLinearVel(math::Vector3(vel_linear_x, vel_linear_y, vel_linear_z));

      }



      public: void OnRosMsg_x_speed(const std_msgs::Float32ConstPtr &_msg)
      {
        this->vel_linear_x = _msg->data;
      }


      public: void OnRosMsg_y_speed(const std_msgs::Float32ConstPtr &_msg)
      {
        this->vel_linear_y = _msg->data;
      }



      private: physics::ModelPtr model; // Pointer to the model
      private: common::Time updatePeriod;
      private: common::Time prevUpdate;


      private: event::ConnectionPtr updateConnection;   // Pointer to the update event connection
      private: std::string link_name;

      private:
          int obstacle_index;

      private:
        float vel_linear_x = 0;
        float vel_linear_y = 0;
        float vel_linear_z = 0;
      private:
        bool flag;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

