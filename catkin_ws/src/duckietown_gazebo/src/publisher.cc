/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <iostream>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::setupClient(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish Gazebo topics:
  // Modify poses for ligths
  gazebo::transport::PublisherPtr pubPose = node->Advertise<gazebo::msgs::Pose>("~/pose_example");
  
  gazebo::transport::PublisherPtr pubVisual = node->Advertise<gazebo::msgs::Visual>("/gazebo/defualt/visual");

  gazebo::msgs::Visual visualMsg;
  visualMsg.set_name("__LED__");
  visualMsg.set_parent_name("led_1_link");
  visualMsg.mutable_material()->mutable_script()->set_name("Gazebo/RedGlow");

  pubVisual->Publish(visualMsg);
  // Turn on and off lights
  // This is for gazebo > 7 gazebo::transport::PublisherPtr pubLight = node->Advertise<gazebo::msgs::Light>("/gazebo/default/light/modify");
  gazebo::transport::PublisherPtr pubLight = node->Advertise<gazebo::msgs::Light>("/gazebo/default/light");

  // Wait for a subscriber to connect
  pubLight->WaitForConnection();

  int value = 0;
  int position = 0;
  // Publisher loop...replace with your own code.
  while (true)
  {
    // Throttle Publication
    gazebo::common::Time::MSleep(100);


    gazebo::msgs::Color diffuse;
    diffuse.set_r(value);
    diffuse.set_g(value);
    diffuse.set_b(value);
    diffuse.set_a(value);
    
    gazebo::msgs::Light msg;
    msg.set_name("user_spot_light_1");
    msg.mutable_diffuse()->CopyFrom(diffuse);

    pubLight->Publish(msg);
    
    value++;

    if(value>255)
    {
      value = 0;

      // Generate a pose
      gazebo::math::Pose pose(position, 0, 2, 0, 0, 0);
      position++;
      gazebo::msgs::Light msg1;
      msg1.set_name("user_spot_light_1");
      gazebo::msgs::Set(msg1.mutable_pose(), pose);

    //      msg1.mutable_diffuse()->CopyFrom(diffuse);
      // Convert to a pose message
//      gazebo::msgs::Pose msg;
//      gazebo::msgs::Set(&msg, pose);

      pubLight->Publish(msg1);

    }
  }

  // Make sure to shut everything down.
  gazebo::shutdown();
}
