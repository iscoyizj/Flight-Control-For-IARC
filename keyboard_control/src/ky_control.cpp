#include <termio.h>
#include <pthread.h>
#include <iostream>
#include <math.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

string cmd_str[] = {"turn_left", "turn_right", "left", "right", "forward", "backward", "up", "down"};
char cmd_char[] = {'7', '9', '4', '6', '1', '3', '8', '2'};
int cmd_num = 8;

mavros_msgs::State currentState;
geometry_msgs::PoseStamped localPose;
geometry_msgs::PoseStamped targetPose;

static pthread_mutex_t mutex;

void euler2quaternion(double* eular, double* quaternion) {
	double roll = eular[0];
	double pitch = eular[1];
	double yaw = eular[2];
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double qw = cy * cr * cp + sy * sr * sp;
    double qx = cy * sr * cp - sy * cr * sp;
    double qy = cy * cr * sp + sy * sr * cp;
    double qz = sy * cr * cp - cy * sr * sp;
	quaternion[0] = qx;
	quaternion[1] = qy;
	quaternion[2] = qz;
	quaternion[3] = qw;
}

void quaternion2euler(double* quaternion, double* eular) {
	double qx = quaternion[0];
	double qy = quaternion[1];
	double qz = quaternion[2];
	double qw = quaternion[3];
	double roll = std::atan2(2 * (qw * qx + qy * qz), (1 - 2 * (qx * qx + qy * qy)));
    double pitch = std::asin(2 * (qw * qy - qx * qz));
    double yaw = std::atan2(2 * (qw * qz + qx * qy), (1 - 2 * (qz * qz + qy * qy)));
	eular[0] = roll;
	eular[1] = pitch;
	eular[2] = yaw;
}

void voice_handler(const std_msgs::String::ConstPtr& msg) {
	double eular[3];
	double quaternion[4];
	string cmd = msg->data;
	cout<<msg->data<<endl;
	int idx = -1;
	for(int i = 0; i < cmd_num; ++i) {
		if(cmd == cmd_str[i]) {
			idx = i;
			break;
		}
	}
	if(idx == -1) return;
	switch(cmd_char[idx]) {		
		pthread_mutex_lock(&mutex);
		case '1':
			cout << ":forward" <<endl;
			targetPose.pose.position.y = targetPose.pose.position.y + 0.5;
			break;
		case '3':
			cout << ":backward" <<endl;
			targetPose.pose.position.y = targetPose.pose.position.y - 0.5;
			break;
		case '4':
			cout << ":left" <<endl;
			targetPose.pose.position.x = targetPose.pose.position.x + 0.5;
			break;
		case '6':
			targetPose.pose.position.x = targetPose.pose.position.x - 0.5;
			cout << ":right" <<endl;
			break;
		case '8':
			cout << ":up" <<endl;
			targetPose.pose.position.z = targetPose.pose.position.z + 0.5;
			break;
		case '2':
			cout << ":down" <<endl;
			targetPose.pose.position.z = targetPose.pose.position.z - 0.5;
			break;
		case '7':
			cout << ":counterclockwise" <<endl;
			quaternion[0] = targetPose.pose.orientation.x;
			quaternion[1] = targetPose.pose.orientation.y;
			quaternion[2] = targetPose.pose.orientation.z;
			quaternion[3] = targetPose.pose.orientation.w;
			quaternion2euler(quaternion, eular);
			eular[2] = eular[2] + 0.5; //about 11 degree
			euler2quaternion(eular, quaternion);
			targetPose.pose.orientation.x = quaternion[0];
			targetPose.pose.orientation.y = quaternion[1];
			targetPose.pose.orientation.z = quaternion[2];
			targetPose.pose.orientation.w = quaternion[3];				
			break;
		case '9':
			cout << ":clockwise" <<endl;
			quaternion[0] = targetPose.pose.orientation.x;
			quaternion[1] = targetPose.pose.orientation.y;
			quaternion[2] = targetPose.pose.orientation.z;
			quaternion[3] = targetPose.pose.orientation.w;
			quaternion2euler(quaternion, eular);
			eular[2] = eular[2] - 0.5; //about 11 degree
			euler2quaternion(eular, quaternion);
			targetPose.pose.orientation.x = quaternion[0];
			targetPose.pose.orientation.y = quaternion[1];
			targetPose.pose.orientation.z = quaternion[2];
			targetPose.pose.orientation.w = quaternion[3];
			break;
		case '5':
			cout << ":no move" <<endl;
			break;
		pthread_mutex_lock(&mutex);
	}
}

void stateCb(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
}

void getLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    localPose = *msg;
}

int scanKeyboard() {
	int in;
	struct termios new_settings;
	struct termios stored_settings;
	tcgetattr(0,&stored_settings);
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0,&stored_settings);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&new_settings);

	in = getchar();
 
	tcsetattr(0,TCSANOW,&stored_settings);
	return in;
}

void* getCmd(void *dummy) {
	cout<<"cmd thread running"<<endl;
	double eular[3];
	double quaternion[4];
	while(true) {
		switch((char)scanKeyboard()){		
			pthread_mutex_lock(&mutex);
			case '1':
				cout << ":forward" <<endl;
				targetPose.pose.position.y = targetPose.pose.position.y + 0.2;
				break;
			case '3':
				cout << ":backward" <<endl;
				targetPose.pose.position.y = targetPose.pose.position.y - 0.2;
				break;
			case '4':
				cout << ":left" <<endl;
				targetPose.pose.position.x = targetPose.pose.position.x + 0.2;
				break;
			case '6':
				targetPose.pose.position.x = targetPose.pose.position.x - 0.2;
				cout << ":right" <<endl;
				break;
			case '8':
				cout << ":up" <<endl;
				targetPose.pose.position.z = targetPose.pose.position.z + 0.2;
				break;
			case '2':
				cout << ":down" <<endl;
				targetPose.pose.position.z = targetPose.pose.position.z - 0.2;
				break;
			case '7':
				cout << ":counterclockwise" <<endl;
				quaternion[0] = targetPose.pose.orientation.x;
				quaternion[1] = targetPose.pose.orientation.y;
				quaternion[2] = targetPose.pose.orientation.z;
				quaternion[3] = targetPose.pose.orientation.w;
				quaternion2euler(quaternion, eular);
				eular[2] = eular[2] + 0.2; //about 11 degree
				euler2quaternion(eular, quaternion);
				targetPose.pose.orientation.x = quaternion[0];
				targetPose.pose.orientation.y = quaternion[1];
				targetPose.pose.orientation.z = quaternion[2];
				targetPose.pose.orientation.w = quaternion[3];				
				break;
			case '9':
				cout << ":clockwise" <<endl;
				quaternion[0] = targetPose.pose.orientation.x;
				quaternion[1] = targetPose.pose.orientation.y;
				quaternion[2] = targetPose.pose.orientation.z;
				quaternion[3] = targetPose.pose.orientation.w;
				quaternion2euler(quaternion, eular);
				eular[2] = eular[2] - 0.2; //about 11 degree
				euler2quaternion(eular, quaternion);
				targetPose.pose.orientation.x = quaternion[0];
				targetPose.pose.orientation.y = quaternion[1];
				targetPose.pose.orientation.z = quaternion[2];
				targetPose.pose.orientation.w = quaternion[3];
				break;
			case '5':
				cout << ":no move" <<endl;
				break;
			pthread_mutex_lock(&mutex);
		}
	}
}

bool isReached(geometry_msgs::PoseStamped currentPose, geometry_msgs::PoseStamped targetPose, 
			double distanceThres, double angleThres) {
	double distance = (currentPose.pose.position.x-targetPose.pose.position.x) 
					* (currentPose.pose.position.x-targetPose.pose.position.x) 
					+ (currentPose.pose.position.y-targetPose.pose.position.y) 
					* (currentPose.pose.position.y-targetPose.pose.position.y) 
					+ (currentPose.pose.position.z-targetPose.pose.position.z) 
					* (currentPose.pose.position.z-targetPose.pose.position.z);
	distance = sqrt(distance);
	if(distance < distanceThres)
		return true;
	return false;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ky_node");
    ros::NodeHandle nh;
	
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, stateCb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, getLocalPose);

	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
            
    ros::Subscriber voice_sub = nh.subscribe<std_msgs::String>
            ("voiceorder", 10, voice_handler);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !currentState.connected){
        ros::spinOnce();
        rate.sleep();
    }

	geometry_msgs::PoseStamped initPose;
	initPose.header.frame_id = "map";
    initPose.pose.position.x = 0;
    initPose.pose.position.y = 0;
    initPose.pose.position.z = 1;

	targetPose = initPose;

	//send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(initPose);
        ros::spinOnce();
        rate.sleep();
    }

	mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

	pthread_mutex_init(&mutex, NULL);
	pthread_t tid;
	void* dummy;
	pthread_create(&tid, NULL, getCmd, dummy);
	
    while(ros::ok()) {
        if( currentState.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !currentState.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

		local_pos_pub.publish(targetPose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

