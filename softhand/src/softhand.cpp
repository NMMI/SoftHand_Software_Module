#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include "definitions.h"	
#include "qbmove_communications.h"	

using namespace std;

// Macro definitions

#define MAX_HAND_CL 18000

double 	ref_hand(0);
bool		ref_hand_changed = false;

/*---------------------------------------------------------------------*
*                                                *
*                                                                      *
*----------------------------------------------------------------------*/
void ref_hand__Callback(const std_msgs::Float64::ConstPtr& msg){
	ref_hand = msg->data;
	ref_hand_changed = true;
}


int main(int argc, char **argv)
{   
  comm_settings comm_settings_sh;
  short int 	inputs[2];
	short int 	measurements[3];

	string 			port;
	string 			ref_hand_topic;
	string 			meas_hand_topic;

	double 			run_freq(100);			// Desired rate for hand position reading and setting
	int					IDhand;

  ros::init(argc,argv,"softhand");  // Initiate new ROS node

	// NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
	ros::NodeHandle n;  

	n.getParam("port", port);
	n.getParam("ID", IDhand);
	n.getParam("run_freq", run_freq);
	n.getParam("ref_hand_topic", ref_hand_topic);
	n.getParam("meas_hand_topic", meas_hand_topic);

	ros::Publisher 	pub_meas_hand	= n.advertise<std_msgs::Float64MultiArray>(meas_hand_topic, 1);
	ros::Subscriber sub_ref_hand	= n.subscribe(ref_hand_topic, 1, ref_hand__Callback);

	ros::Rate loop_rate(run_freq);

	// Device preparation
	cout << "\nROS NODE FOR SOFTHAND WITH ID " << IDhand << " \n";
	// Opening the port the device is connected to
	cout << "Opening COM port to communicate with the SoftHand.\n";

	openRS485(&comm_settings_sh, port.c_str());

	// Activate softhand motor
	cout << "Activating SoftHand motor.\n";
	ros::Duration(1).sleep();
	commActivate(&comm_settings_sh, IDhand, 1);
	ros::Duration(0.05).sleep();

	cout << "\nYou can now communicate with SoftHand using \'" << 
		ref_hand_topic << "\' topic to give [0.0, 1.0] closure reference and \'" << 
		meas_hand_topic << "\' topic to retrieve its actual position always in the [0.0, 1.0] range.\n";
	std_msgs::Float64MultiArray 	msg_meas_hand;

	while ( ros::ok() )
	{

		/**************** SoftHand position reading and setting section ***********************/

		msg_meas_hand.data.clear();

		// Small break to allow multiple consecutive COM readings
		ros::Duration(0.00025).sleep();		//250 us sleep
	
		if(commGetMeasurements(&comm_settings_sh, IDhand, measurements)>0){
    //    cout << "Measurements: " << measurements[0] << " " << measurements[1] << " " << measurements[2] << endl;
			msg_meas_hand.data.push_back( ((float)measurements[0])/MAX_HAND_CL);
		}

		// Publish softhand position
		pub_meas_hand.publish(msg_meas_hand);


		if (ref_hand_changed){
			inputs[0] = ref_hand*MAX_HAND_CL;
	  	inputs[1] = 0;

	  	// Small break to allow multiple consecutive COM readings
			ros::Duration(0.00025).sleep();		//250 us sleep
	  		
	  	commSetInputs(&comm_settings_sh, IDhand, inputs);

			ref_hand_changed = false;
		}

		ros::spinOnce();     // Need to call this function often to allow ROS to process incoming messages 
		loop_rate.sleep();   // Sleep for the rest of the cycle, to enforce the loop rate
		
	}

	commActivate(&comm_settings_sh, IDhand, 0);
	ros::Duration(0.05).sleep();
	closeRS485(&comm_settings_sh);
	return 0;

}