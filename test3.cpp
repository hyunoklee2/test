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

	  physics::Link_V links = model->GetLinks();
      physics::Collision_V colls;
      for (unsigned int i = 0; i < links.size(); i++) {
		  
		  if (links[i]->GetName().compare(light_r) == 0){
		  		light_red_pose = links[i]->GetWorldPose() ;		  	
		  	}else if (links[i]->GetName().compare(light_g) == 0){
		  		light_green_pose = links[i]->GetWorldPose() ;		  	
		  	}else if (links[i]->GetName().compare(light_y) == 0){
		  		light_yellow_pose = links[i]->GetWorldPose() ;		  	
		  	}else if (links[i]->GetName().compare(traffic_bar) == 0){
		  		traffic_bar_pose = links[i]->GetWorldPose() ;
				traffic_bar_pose2 = math::Pose( traffic_bar_pose.pos.x,traffic_bar_pose.pos.y,traffic_bar_pose.pos.z
					,traffic_bar_pose.rot.z,traffic_bar_pose.rot.w,traffic_bar_pose.rot.x ) ;
		  	}

          cout << links[i]->GetName() << "\t\t" <<links[i]->GetWorldPose() << endl;
          colls = links[i]->GetCollisions();
          for(unsigned i=0; i<colls.size(); i++)
          {
              colls[i]->Update();
              cout << colls[i]->GetName() << "\t\t" << colls[i]->GetWorldPose() << endl;
          }

      }

	  
	  
	  
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
			
				this->model->SetLinkWorldPose( traffic_bar_pose), std::string(traffic_bar));
				bar_state = off ;
			
	  		}else if(bar_state==off){
	  		    
		  		this->model->SetLinkWorldPose( traffic_bar_pose2), std::string(traffic_bar));
				bar_state = on ;
			
	  		}
			bar_set_time = cur_time;
			
	  	}	  


	  if( light_time_diff > light_duration ){
	  	
	  	if(light_state==red){
			
				light_state = green ;
				this->model->SetLinkWorldPose( light_green_pose, std::string(light_g));
				this->model->SetLinkWorldPose( dispresent, std::string(light_r));
				this->model->SetLinkWorldPose( dispresent2, std::string(light_y));
				
			
	  		}else if(light_state==green){
	  		
				light_state = yellow ;
				this->model->SetLinkWorldPose( light_yellow_pose, std::string(light_y));
				this->model->SetLinkWorldPose( dispresent, std::string(light_r));
				this->model->SetLinkWorldPose( dispresent2, std::string(light_g));
			
	  		}else if(light_state==yellow){
				
				light_state = red ;
				this->model->SetLinkWorldPose( light_red_pose, std::string(light_r));
				this->model->SetLinkWorldPose( dispresent, std::string(light_g));
				this->model->SetLinkWorldPose( dispresent2, std::string(light_y));
			
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

	string light_r = "light_r";
	string light_g = "light_g";
	string light_y = "light_y";
	string traffic_bar = "traffic_bar";

	math::Pose light_red_pose = math::Pose( 0,0,0,0,0,0 ) ;
	math::Pose light_green_pose = math::Pose( 0,0,0,0,0,0 ) ;
	math::Pose light_yellow_pose = math::Pose( 0,0,0,0,0,0 ) ;
	math::Pose traffic_bar_pose = math::Pose( 0,0,0,0,0,0 ) ;
	math::Pose traffic_bar_pose2 = math::Pose( 0,0,0,0,0,0 ) ;
	math::Pose dispresent = math::Pose( 0,0,-5,0,0,0 ) ;
	math::Pose dispresent2 = math::Pose( 0,0,-9,0,0,0 ) ;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
