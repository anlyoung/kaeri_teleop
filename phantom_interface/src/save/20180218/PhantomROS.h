#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>
#include <fstream>                      //file write load

using namespace std;

struct OmniState {
    HLdouble position[3];
    HLdouble orientation[4];
    HLdouble scale;
    vector<HLdouble> mesh_buffer;
    HLint nVertex;

    HLboolean button1;
    HLboolean button2;

    bool save;
    std::vector<double> save_x;
    std::vector<double> save_y;
    std::vector<double> save_z;
};

/* ROS class. */
class PhantomROS {
private:
    OmniState *state;

    ros::Publisher joint_pub;
    ros::Publisher button1_pub;
    ros::Publisher button2_pub;
    ros::Subscriber mesh_sub;

public:
    ros::NodeHandle n;

    PhantomROS(OmniState *s){
        state = s;
        state->scale = 0.5;
	state->save = false;

        // Publish joint states on "Phantom_joint_states"
        joint_pub = n.advertise<geometry_msgs::Pose>("Phantom_joint_states", 1);
        // Publish button state on "Phantom_button"
        button1_pub = n.advertise<std_msgs::Bool>("Phantom_button1", 100);
        button2_pub = n.advertise<std_msgs::Bool>("Phantom_button2", 100);
        
        // subscribe mesh data on "Phantom_mesh"
        mesh_sub = n.subscribe("Phantom_mesh", 100, &PhantomROS::stl_update_callback, this);
        ROS_INFO("Initialized");
    }

    void stl_update_callback(const std_msgs::Float64MultiArray& stl) {
        int size = stl.layout.dim[0].size;
        state-> nVertex = stl.layout.dim[0].stride;
        //ROS_INFO("%d %d %d %d", stl.layout.dim[0].size, stl.layout.dim[1].size, stl.layout.dim[0].stride, stl.layout.dim[1].stride);

        state->mesh_buffer.clear();

        for (int i=0;i<size;i++){
            state->mesh_buffer.push_back(stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i]/state->scale);
            state->mesh_buffer.push_back(stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i+1]/state->scale);
            state->mesh_buffer.push_back(stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i+2]/state->scale);
						/*		        
						if(i<123)
            ROS_INFO("%d: %f %f %f",
										i,
                    state->mesh_buffer[i*3],
                    state->mesh_buffer[i*3+1],
                    state->mesh_buffer[i*3+2]);
						*/
        }

        ROS_INFO("%d Vertices updated", size);
    }

    void publish_omni_state() {
        geometry_msgs::Pose cursor_pose;
        cursor_pose.position.x = state->position[0]*state->scale;
        cursor_pose.position.y = state->position[1]*state->scale;
        cursor_pose.position.z = state->position[2]*state->scale;

        cursor_pose.orientation.x = state->orientation[0];
        cursor_pose.orientation.y = state->orientation[1];
        cursor_pose.orientation.z = state->orientation[2];
        cursor_pose.orientation.w = state->orientation[3];

        joint_pub.publish(cursor_pose);

        //ROS_INFO("Real  : %f, %f, %f", state->position[0], state->position[1], state->position[2]);
        //ROS_INFO("Scaled: %f, %f, %f", joint_state.position[0], joint_state.position[1], joint_state.position[2]);

        std_msgs::Bool button1, button2;
        button1.data = state->button1;
        button2.data = state->button2;
        button1_pub.publish(button1);
        button2_pub.publish(button2);
        //ROS_INFO("%d, %d", state->button1, state->button2);

	if (state->button1){
	    state->save = true;
	    state->save_x.push_back(cursor_pose.position.x);
	    state->save_y.push_back(cursor_pose.position.y);
 	    state->save_z.push_back(cursor_pose.position.z);	
	}
	
	if (state->button2){
	    if (state->save){
		state->save = false;
		ofstream saveFile ("/home/user/catkin_ws/src/omni_test/trajectory.txt");
		for (int i = 0; i<state->save_x.size(); i++){
		    saveFile << state->save_x[i] << "\t" << state->save_y[i] << "\t" << state->save_z[i] << "\n";		
		}
		saveFile.close();
        	ROS_INFO("trajectory saved");
	    }
	}
    }
};
