// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h> 
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// STL
#include <vector>
#include <fstream>                      //file write load

// Boost
#include <boost/thread/mutex.hpp>

using namespace std;

struct OmniState {
    HLdouble position[3];
    HLdouble orientation[4];
    HLdouble scale;
    vector<HLdouble> mesh_buffer;
    HLint nVertex;

    HLboolean button1;
    HLboolean button2;
		HLboolean touched;

    bool save;
    std::vector<double> save_x;
    std::vector<double> save_y;
    std::vector<double> save_z;
};

/* ROS class. */
class PhantomROS {
	private:
    ros::NodeHandle m_nh;		

		boost::mutex &m_mesh_mutex;

    OmniState *state;

    ros::Publisher joint_pub;
    ros::Publisher button_pub;
		ros::Publisher touch_pub;

    ros::Subscriber mesh_sub;
		ros::Subscriber touch_mode_sub;

		tf::TransformBroadcaster tf_br;

		geometry_msgs::Pose cs_pose_world;
		geometry_msgs::Pose cs_pose_phtm;
		tf::Transform tf_phtm_misalignment;
		tf::Transform tf_phtm_rviz;
		tf::Transform tf_phtm;

		bool touch_mode;

		bool button1;
		bool button1_save;
		int button1_drag;
		geometry_msgs::Pose button1_drag_start_pt;
		geometry_msgs::Pose button1_drag_current_pt;
		geometry_msgs::Pose button1_drag_end_pt;
		bool button2;
		bool button2_save;
		int button2_drag;
		geometry_msgs::Pose button2_drag_start_pt;
		geometry_msgs::Pose button2_drag_current_pt;
		geometry_msgs::Pose button2_drag_end_pt;
		bool buttons_save;
		int buttons_drag;
		geometry_msgs::Pose buttons_drag_start_pt;
		geometry_msgs::Pose buttons_drag_current_pt;
		geometry_msgs::Pose buttons_drag_end_pt;

		std_msgs::String btn_msg_save; 

		#define DRAG_TIME 		30
		#define PI						3.14159265359	


	public:
    PhantomROS(ros::NodeHandle &nh, boost::mutex &mesh_mutex, OmniState *s): m_nh(nh), m_mesh_mutex(mesh_mutex)
		{
      state = s;
      state->scale = 0.5;
			state->save = false;
			state->touched = false;

			// Publishers
      joint_pub = m_nh.advertise<geometry_msgs::Pose>("Phantom_joint_states", 1);
      button_pub = m_nh.advertise<std_msgs::String>("Phantom_button_state", 1);
      touch_pub = m_nh.advertise<std_msgs::Bool>("Phantom_touch_state", 1);

      // Subscribers
			touch_mode_sub = m_nh.subscribe("touch_mode", 1, &PhantomROS::touch_mode_callback, this);
      mesh_sub = m_nh.subscribe("Phantom_mesh", 1, &PhantomROS::mesh_update_callback, this);

			cs_pose_phtm = geometry_msgs::Pose();
			cs_pose_world = geometry_msgs::Pose();

			// tf for frame misalignment between phantom device and openhaptics coordinate
			tf::Quaternion rot1;
			rot1.setEuler(-PI/2, 0, 0);
			tf::Quaternion rot2;
			rot2.setEuler(0, PI/2, 0);

			rot2 *= rot1;
			tf_phtm_misalignment = tf::Transform(rot2);
	
			// tf for phantom center in rviz virtual space (world)
			tf::Vector3 pos_tmp(1.0, 0.0, 0.4);
			tf::Quaternion ori_tmp(0.0, 0.0, 0.0, 1.0);
			tf_phtm_rviz = tf::Transform(ori_tmp, pos_tmp);
	
			// tf for openhaptics coordinate in rviz virtual space (world)
			tf_phtm.setOrigin(tf_phtm_rviz.getOrigin());
			tf_phtm.setRotation(tf_phtm_rviz*tf_phtm_misalignment.getRotation());

			// touch mode on
			touch_mode = true;

			// Button state
			button1 = false;
			button1_save = false;
			button1_drag = 0;
			button1_drag_start_pt;
			button1_drag_current_pt = geometry_msgs::Pose();
			button1_drag_end_pt = geometry_msgs::Pose();
			button2 = false;
			button2_save = false;
			button2_drag = 0;
			button2_drag_start_pt = geometry_msgs::Pose();
			button2_drag_current_pt = geometry_msgs::Pose();
			button2_drag_end_pt = geometry_msgs::Pose();
			buttons_save = false;
			buttons_drag = 0;
			buttons_drag_start_pt = geometry_msgs::Pose();
			buttons_drag_current_pt = geometry_msgs::Pose();
			buttons_drag_end_pt = geometry_msgs::Pose();

      ROS_INFO("[PhantomROS]Initialized");
    }


    void mesh_update_callback(const geometry_msgs::PoseArray::ConstPtr& msg) {
			boost::mutex::scoped_lock lock(m_mesh_mutex);
			if(touch_mode){
				int size = msg->poses.size();
		    state-> nVertex = size;

		    state->mesh_buffer.clear();

		    for (int i = 0; i < size; i++){
		        state->mesh_buffer.push_back(msg->poses[i].position.x/state->scale);
		        state->mesh_buffer.push_back(msg->poses[i].position.y/state->scale);
		        state->mesh_buffer.push_back(msg->poses[i].position.z/state->scale);
		    }
		    //ROS_INFO("%d Vertices updated", size);
			}
			else{
		    state-> nVertex = 0;
		    state->mesh_buffer.clear();
				//std::cout << "buffer cleared" << std::endl;
			}
    }


		void touch_mode_callback(const std_msgs::Bool::ConstPtr& msg) {
			touch_mode = msg->data;
			std::cout << touch_mode << std::endl;
    }


    void publish_omni_state() {
			// transform of cursor point to world frame
      geometry_msgs::Pose cs_pose_raw;
      cs_pose_raw.position.x = state->position[0]*state->scale;
      cs_pose_raw.position.y = state->position[1]*state->scale;
      cs_pose_raw.position.z = state->position[2]*state->scale;
			
			geometry_msgs::Pose tmp;
			tmp.orientation.x = state->orientation[0];
			tmp.orientation.y = state->orientation[1];
			tmp.orientation.z = -state->orientation[2];
			tmp.orientation.w = state->orientation[3];				
			//ROS_INFO("%f, %f, %f, %f", tmp.orientation.x, tmp.orientation.y, tmp.orientation.z, tmp.orientation.w);

      cs_pose_raw.orientation.x = 1.0/sqrt(2.0)*(tmp.orientation.x+tmp.orientation.z);
      cs_pose_raw.orientation.y = 1.0/sqrt(2.0)*(tmp.orientation.y+tmp.orientation.w);
      cs_pose_raw.orientation.z = 1.0/sqrt(2.0)*(tmp.orientation.z-tmp.orientation.x);
      cs_pose_raw.orientation.w = 1.0/sqrt(2.0)*(tmp.orientation.w-tmp.orientation.y);
			
			tf::Vector3 pos_tmp(cs_pose_raw.position.x, cs_pose_raw.position.y, cs_pose_raw.position.z);
			cs_pose_phtm.position.x = (tf_phtm_misalignment*pos_tmp).x();
			cs_pose_phtm.position.y = (tf_phtm_misalignment*pos_tmp).y();
			cs_pose_phtm.position.z = (tf_phtm_misalignment*pos_tmp).z();
			tf::Quaternion ori_tmp(cs_pose_raw.orientation.x, cs_pose_raw.orientation.y, cs_pose_raw.orientation.z, cs_pose_raw.orientation.w);
			ori_tmp.normalize();
			cs_pose_phtm.orientation.x = (tf_phtm_misalignment*ori_tmp).x();
			cs_pose_phtm.orientation.y = (tf_phtm_misalignment*ori_tmp).y();
			cs_pose_phtm.orientation.z = (tf_phtm_misalignment*ori_tmp).z();
			cs_pose_phtm.orientation.w = (tf_phtm_misalignment*ori_tmp).w();

			tf::Vector3 pos_tmp2(cs_pose_phtm.position.x, cs_pose_phtm.position.y, cs_pose_phtm.position.z);
			cs_pose_world.position.x = (tf_phtm_rviz*pos_tmp2).x();
			cs_pose_world.position.y = (tf_phtm_rviz*pos_tmp2).y();
			cs_pose_world.position.z = (tf_phtm_rviz*pos_tmp2).z();
			tf::Quaternion ori_tmp2(cs_pose_phtm.orientation.x, cs_pose_phtm.orientation.y, cs_pose_phtm.orientation.z, cs_pose_phtm.orientation.w);
			ori_tmp2.normalize();
			cs_pose_world.orientation.x = (tf_phtm_rviz*ori_tmp2).x();		
			cs_pose_world.orientation.y = (tf_phtm_rviz*ori_tmp2).y();
			cs_pose_world.orientation.z = (tf_phtm_rviz*ori_tmp2).z();
			cs_pose_world.orientation.w = (tf_phtm_rviz*ori_tmp2).w();
			
			// tf broadcast
			ros::Time now = ros::Time::now();
			tf_br.sendTransform(tf::StampedTransform(tf_phtm, now, "world", "phantom"));
			tf_br.sendTransform(tf::StampedTransform(tf_phtm_rviz, now, "world", "phantom_rviz"));
			
			// publish cs_pose_world
			joint_pub.publish(cs_pose_world);
			//ROS_INFO("%f, %f, %f", cs_pose_world.position.x, cs_pose_world.position.y, cs_pose_world.position.z);
			//ROS_INFO("%f, %f, %f, %f", cs_pose_world.orientation.x, cs_pose_world.orientation.y, cs_pose_world.orientation.z, cs_pose_world.orientation.w);

			// publish touch
			std_msgs::Bool touched;
			touched.data = state->touched;
			touch_pub.publish(touched);

			// publish button state
			button1 = state->button1;
			button2 = state->button2;
			std_msgs::String btn_msg = buttonState();
			if ((btn_msg_save.data != btn_msg.data)/*||(btn_msg.data == "btn1drag")||(btn_msg.data == "btn2drag")*/){
				button_pub.publish(btn_msg);
				btn_msg_save = btn_msg;
				std::cout << btn_msg_save.data << std::endl;
			}
			
			// move phantom center
			if (btn_msg.data == "btnsdown" || btn_msg.data == "btnsdrag"){
				tf_phtm_rviz.setOrigin(tf::Vector3(cs_pose_world.position.x, cs_pose_world.position.y, cs_pose_world.position.z));
				tf_phtm.setOrigin(tf_phtm_rviz.getOrigin());
				tf_phtm.setRotation(tf_phtm_rviz*tf_phtm_misalignment.getRotation());
			}
    }


		std_msgs::String buttonState(){
			std_msgs::String btn_state;
			// (buttons down both) or (any button stays down after both buttons down)
			if ((button1 and button2) or (button1 and buttons_save) or (button2 and buttons_save)){	
				button1_save = false;
				button1_drag = 0;
				button2_save = false;
				button2_drag = 0;
				if (buttons_save){			
					buttons_drag = buttons_drag+1;
					if (buttons_drag > DRAG_TIME){
						buttons_drag_current_pt.position = cs_pose_phtm.position;
						btn_state.data = "btnsdrag";
						return btn_state;		// buttons dragging
					}
					else{
						btn_state.data = "nothing";
						return btn_state;		// wait for dragging
					}
				}
				else{							
					buttons_save = true;
					buttons_drag_start_pt.position = cs_pose_phtm.position;
					btn_state.data = "btnsdown";
					return btn_state;			// buttons just down
				}
			}

			// check button release (no button down)
			if (buttons_save){
				buttons_save = false;	
				// save end point if dragging and reset start point
				if (buttons_drag > DRAG_TIME) buttons_drag_end_pt.position = cs_pose_phtm.position;
				buttons_drag_start_pt = geometry_msgs::Pose();
				buttons_drag = 0;
				btn_state.data = "btnsup";
				return btn_state;				// buttons just released				
			}

			// button 1 down
			if (button1){ 					
				if (button1_save){
					button1_drag = button1_drag+1;
					if (button1_drag > DRAG_TIME){
						button1_drag_current_pt = cs_pose_phtm;
						btn_state.data = "btn1drag";						
						return btn_state;		// button1 dragging
					}
					else{
						btn_state.data = "nothing";
						return btn_state;		// wait for dragging
					}	
				}
				else{ 									// button1 just down
					button1_save = true;
					button1_drag_start_pt = cs_pose_phtm;
					btn_state.data = "btn1down";
					return btn_state;
				}
			}

			// button 2 down
			if (button2){ 					
				if (button2_save){ 			
					button2_drag = button2_drag+1;
					if (button2_drag > DRAG_TIME){
						button2_drag_current_pt = cs_pose_phtm;
						btn_state.data = "btn2drag";
						return btn_state;		// button2 dragging
					}
					else{
						btn_state.data = "nothing";
						return btn_state;		// wait for dragging
					}
				}	
				else{ 							
					button2_save = true;
					button2_drag_start_pt = cs_pose_phtm;
					btn_state.data = "btn2down";
					return btn_state;			// button2 just down
				}
			}

			// button 1 released
			if (button1_save){
				if (button1_drag > DRAG_TIME){
					button1_drag_end_pt = cs_pose_phtm;
					button1_drag_start_pt = geometry_msgs::Pose();
					button1_drag = 0;
					button1_save = false;
					btn_state.data = "nothing";		
					return btn_state;
				}	
				else{
					button1_drag_start_pt = geometry_msgs::Pose();
					button1_drag = 0;
					button1_save = false;				
					btn_state.data = "btn1up";					
					return btn_state;					// button1 just released
				}
			}

			// button 2 released
			if (button2_save){
				if (button2_drag > DRAG_TIME){
					button2_drag_end_pt = cs_pose_phtm;
					button2_drag_start_pt = geometry_msgs::Pose();
					button2_drag = 0;
					button2_save = false;
					btn_state.data = "nothing";
					return btn_state;
				}
				else{
					button2_drag_start_pt = geometry_msgs::Pose();
					button2_drag = 0;
					button2_save = false;
					btn_state.data = "btn2up";
					return btn_state;					// button2 just released
				}
			}

			// nothing happened
			btn_state.data = "nothing";
			return btn_state;
		}
};
