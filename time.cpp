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
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ctime>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));
	  bar_set_time = time(NULL);
	  
	  
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      
	  double     bar_time_diff;
	  double     light_time_diff;
      time_t cur_time = time(NULL) ;
	  bar_time_diff = difftime( cur_time, bar_set_time);
	  light_time_diff = difftime( cur_time, light_set_time);	  

	  if( bar_time_diff > bar_duration ){
	  	
	  	if(bar_state==on){
			
				this->model->SetLinearVel(math::Vector3(.3, 0, 0));
				bar_state = off ;
			
	  		}else if(bar_state==off){
	  		
		  		this->model->SetLinearVel(math::Vector3(.3, 0, 0));
				bar_state = on ;
			
	  		}
			bar_set_time = cur_time;
			
	  	}	  


	  if( light_time_diff > light_duration ){
	  	
	  	if(light_state==red){
			
				this->model->SetLinearVel(math::Vector3(.3, 0, 0));
				light_state = green ;
			
	  		}else if(light_state==green){
	  		
				this->model->SetLinearVel(math::Vector3(.3, 0, 0));
				light_state = yellow ;
			
	  		}else if(light_state==yellow){
				
				this->model->SetLinearVel(math::Vector3(.3, 0, 0));
				light_state = red
			
	  		}
			light_set_time = cur_time;
			
	  	}
	  
      
    }

	
	time_t bar_set_time;
	int on = 0 ;
	int off = 1 ;
	int bar_state = on;
	double bar_duration = 1; // 1sec

	time_t light_set_time;
	int red = 0 ;
	int green = 1 ;
	int yellow = 2 ;
	int light_state = red;
	double light_duration = 1; // 1sec
	

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
