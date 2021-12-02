#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <GL/gl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <pthread.h>

int calibrationStyle;

struct OmniState {
    //hduVector3Dd position;  //3x1 vector of position
    //hduVector3Dd velocity;  //3x1 vector of velocity
    //hduVector3Dd rot;
    //hduVector3Dd joints;
    //hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
    //float thetas[7];
    //int buttons[2];
    //int buttons_prev[2];
    //bool lock;
    //hduVector3Dd lock_pos;

    HLdouble position[3];
    HLdouble scale;

    HLboolean button1;
    HLboolean button2;

    HLuint meshID;
    HLuint effectID;
    HLuint friction;
};

class PhantomROS {
private:
    HDErrorInfo error;
    HHD hHD;
    HHLRC hHLRC;

public:
	ros::NodeHandle n;
	ros::Publisher joint_pub;
    ros::Publisher button1_pub;
    ros::Publisher button2_pub;
    ros::Publisher scale_pub;
    ros::Subscriber mesh_sub;

    std::string omni_name;


	OmniState *state;

    PhantomROS(OmniState *s) {
        hHD = hdInitDevice(HD_DEFAULT_DEVICE);

        if (HD_DEVICE_ERROR(error = hdGetError())) {
            ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
            return;
        }

        ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
        hdEnable(HD_FORCE_OUTPUT);
        //hdStartScheduler();
        //if (HD_DEVICE_ERROR(error = hdGetError())) {
        //    ROS_ERROR("Failed to start the scheduler"); //, &error);
        //    return -1;
        //}
        //HHD_Auto_Calibration();

        hdMakeCurrentDevice(hHD);
        hHLRC = hlCreateContext(hHD);
        hlMakeCurrent(hHLRC);

        hlEnable(HL_HAPTIC_CAMERA_VIEW);

        // initialize variables
        state = s;        
        state->meshID = hlGenShapes(1);
        state->effectID = hlGenEffects(1);
        state->friction = hlGenEffects(1);
        state->scale = 30.0;

        //state->buttons[0] = 0;
        //state->buttons[1] = 0;
        //state->buttons_prev[0] = 0;
        //state->buttons_prev[1] = 0;
        //hduVector3Dd zeros(0, 0, 0);
        //state->velocity = zeros;
        //state->lock = false;
        //state->lock_pos = zeros;
        //state->position[0] = 0.0;
        //state->position[1] = 0.0;
        //state->position[2] = 0.0;

        hlTouchableFace(HL_FRONT_AND_BACK);
        hlEnable(HL_PROXY_RESOLUTION);

        // Publish joint states on "Phantom_joint_states"
        joint_pub = n.advertise<sensor_msgs::JointState>("Phantom_joint_states", 1);
        // Publish button state on "Phantom_button"
        button1_pub = n.advertise<std_msgs::Bool>("Phantom_button1", 100);
        button2_pub = n.advertise<std_msgs::Bool>("Phantom_button2", 100);
        // Publish button state on "Phantom_scale"
        scale_pub = n.advertise<std_msgs::Int32>("Phantom_scale", 100);

        // subscribe mesh data on "Phantom_mesh"
        mesh_sub = n.subscribe("Phantom_mesh", 100, &PhantomROS::stl_update_callback, this);
        ROS_INFO("Initialized");
	}


    ~PhantomROS(){
        hlDeleteShapes(state->meshID, 1);

        if(hHLRC != NULL){
            hlBeginFrame();
            hlStopEffect(state->effectID);
            hlEndFrame();

            hlDeleteEffects(state->effectID, 1);
            hlMakeCurrent(NULL);
            hlDeleteContext(hHLRC);
        }

        if(hHD != HD_INVALID_HANDLE){
            //hdStopScheduler();
            hdMakeCurrentDevice(hHD);
            hdDisableDevice(hHD);
        }
    }


    void stl_update_callback(const std_msgs::Float64MultiArray& stl) {
        // update mesh buffer
        hlBeginFrame();

        hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.7f);
        hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.1f);
        hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.2f);
        hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION, 0.3f);


        // construct virtual object using mesh_pose, phantom_pose
        hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, state->meshID);

        glPushMatrix();
        glBegin(GL_TRIANGLES);
        int size = stl.layout.dim[0].size;
        for (int i=0;i<size;i++){
            glVertex3f(stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i],
                    stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i+1],
                    stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i+2]);
            ROS_INFO("%f %f %f", stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i],
                    stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i+1],
                    stl.data[stl.layout.data_offset+stl.layout.dim[1].stride*i+2]);
        }
        glEnd();
        glPushMatrix();

        hlEndShape();
        glPopMatrix();
        hlEndFrame();

        ROS_INFO("%d Vertices updated", size);
    }


	void publish_omni_state() {
		sensor_msgs::JointState joint_state;
		joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(3);
        joint_state.position.resize(3);
        joint_state.name[0] = "x";
        joint_state.position[0] = state->position[0]/state->scale;
        joint_state.name[1] = "y";
        joint_state.position[1] = state->position[1]/state->scale;
        joint_state.name[2] = "z";
        joint_state.position[2] = state->position[2]/state->scale;

        joint_pub.publish(joint_state);

        //ROS_INFO("%f, %f, %f", state->position[0], state->position[1], state->position[2]);

        std_msgs::Bool button1, button2;
        button1.data = state->button1;
        button2.data = state->button2;
        button1_pub.publish(button1);
        button2_pub.publish(button2);
        //ROS_INFO("%d, %d", state->button1, state->button2);

        std_msgs::Int32 phantom_scale;
        phantom_scale.data = state->scale;
        scale_pub.publish(phantom_scale);

        /*
		if ((state->buttons[0] != state->buttons_prev[0])
				or (state->buttons[1] != state->buttons_prev[1])) {

			if ((state->buttons[0] == state->buttons[1])
					and (state->buttons[0] == 1)) {
				state->lock = !(state->lock);
			}
            std_msgs::Int32MultiArray button_event;
            button_event.data[0] = state->buttons[0];
            button_event.data[1] = state->buttons[1];
			state->buttons_prev[0] = state->buttons[0];
			state->buttons_prev[1] = state->buttons[1];
			button_pub.publish(button_event);
		}
        */
	}

    void startCustomEffect()
    {
        hlBeginFrame();

        hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc) computeEffect, (void*) this);
        hlStartEffect(HL_EFFECT_CALLBACK, state->effectID);

        hlEndFrame();
    }

    static void HLCALLBACK computeEffect(hduVector3Dd &force, HLcache *cache, void *userdata)
    {
        //PhantomROS *hapticDevice = (PhantomROS *)userdata;

        //hlMakeCurrent(hapticDevice->hHLRC);
        //hdMakeCurrentDevice(hapticDevice->hHD);

        //hduVector3Dd proxyPosition;
        //hlCacheGetDoublev(cache, HL_PROXY_POSITION, proxyPosition);

        //hduQuaternion proxyRotationQuaternion;
        //hlCacheGetDoublev(cache, HL_PROXY_ROTATION, proxyRotationQuaternion);

        //HLdouble proxytransform[16];
        //hlCacheGetDoublev(cache, HL_PROXY_TRANSFORM, proxytransform);

        //hduVector3Dd proxyPosNoCache;
        //hlGetDoublev(HL_PROXY_POSITION, proxyPosNoCache);

        //force += hapticDevice->computeCollisionPlaneEffect(proxyPosNoCache);

    }
    /*
    hduVector3Dd computeCollisionPlaneEffect(hduVector3Dd proxyPosition)
    {
        HDdouble dist = collisionPlane.perpDistance(proxyPosition);
        if(!collisionPlaneEnabled && (dist > 0)) {
            collisionPlaneEnabled = true;
        }
        hduVector3Dd force(0.0,0.0,0.0);
        if (collisionPlaneEnabled && (dist < 0)) {
            hduVector3D<HDdouble> planeNormal = collisionPlane.normal();
            planeNormal.normalize();
            force = -planeNormal*dist*planeStiffness;
        }
        return force;
    }
    */
};

int main(int argc, char** argv) {

    // Init ROS
	ros::init(argc, argv, "omni_haptic_node");
	OmniState state;
    PhantomROS omni_ros(&state);

    omni_ros.startCustomEffect();

	// ros thread (publish and listen)
    int publish_rate;
    omni_ros.n.param(std::string("publish_rate"), publish_rate, 100);
    ros::Rate loop_rate(publish_rate);


    while (ros::ok()) {
        hlBeginFrame();
        hlGetDoublev(HL_DEVICE_POSITION, omni_ros.state->position);
        hlGetBooleanv(HL_BUTTON1_STATE, &(omni_ros.state->button1));
        hlGetBooleanv(HL_BUTTON2_STATE, &(omni_ros.state->button2));

        hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.7f);
        hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.1f);
        hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.2f);
        hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION, 0.3f);

        // construct virtual object using mesh_pose, phantom_pose
        glPushMatrix();
        hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, omni_ros.state->meshID);
        glBegin(GL_TRIANGLES);

        glVertex3f(100,0,0);
        glVertex3f(0,0,100);
        glVertex3f(-100,0,0);
        glVertex3f(-100,0,0);
        glVertex3f(0,0,-100);
        glVertex3f(100,0,0);

        glEnd();
        hlEndShape();
        glPopMatrix();



        hlEndFrame();

        omni_ros.publish_omni_state();
        ros::spinOnce();
        loop_rate.sleep();
    }

    //pthread_t ros_thread;
    //pthread_create(&ros_thread, NULL, ros_com, (void*) &omni_ros);
    //pthread_join(ros_thread, NULL);

	ROS_INFO("Ending Session....");


	return 0;
}

/*
void *ros_com(void *ptr) {
    PhantomROS *omni_ros = (PhantomROS *) ptr;
    int publish_rate;
    omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
    ros::Rate loop_rate(publish_rate);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    HLdouble position[3];
    while (ros::ok()) {
        hlGetDoublev(HL_PROXY_POSITION, omni_ros->state->position);
        hlGetBooleanv(HL_BUTTON1_STATE, &(omni_ros->state->button1));
        hlGetBooleanv(HL_BUTTON2_STATE, &(omni_ros->state->button2));
        omni_ros->publish_omni_state();
        loop_rate.sleep();
    }
    return NULL;
}
*/
