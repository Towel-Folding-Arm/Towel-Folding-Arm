#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <unistd.h>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <std_msgs/String.h>
#include <sstream>

using namespace std;

const string PLANNING_GROUP = "arm";
vector<string> joint_names;
vector<double> joint_group_positions;

ros::Publisher PWM_publisher;
geometry_msgs::Pose current_pose;
double home_x, home_y, home_z;

void init() {
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	joint_names = joint_model_group->getVariableNames();

	ROS_INFO_NAMED("Init", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("Init", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	for (size_t i = 0; i < joint_names.size(); ++i) {
		ROS_INFO_NAMED("Init", "Joint %-25s: %f", joint_names[i].c_str(), joint_group_positions[i]);
	}
	joint_group_positions.resize(6);

	current_pose = move_group.getCurrentPose().pose;
	home_x = 0.157065; //current_pose.position.x;
	home_y = 0.004; //current_pose.position.y;
	home_z = 0.18642; //current_pose.position.z;
	ROS_INFO_NAMED("Init", "Starting Position: x: %f y: %f z: %f", home_x, home_y, home_z);

	move_group.setPlanningTime(20.0);
	move_group.setNumPlanningAttempts(100);
}

const double rad = 180.0 / M_PI;
const double big_servo = 750.0/ 90.0 * rad;
const double sml_servo = 850.0 / 90.0 * rad;
const int servos[6] = {1350, 2100, 625, 1400, 1500, 1150};
const double moe = 1e-4;

bool move(double roll, double pitch, double yaw, double x, double y, double z) {
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	tf::Quaternion target_quaternion = tf::createQuaternionFromRPY(roll, pitch, yaw);
	target_quaternion.normalize();

	geometry_msgs::Pose target_pose;
	target_pose.orientation.x = target_quaternion.x();
	target_pose.orientation.y = target_quaternion.y();
	target_pose.orientation.z = target_quaternion.z();
	target_pose.orientation.w = target_quaternion.w();
	target_pose.position.x = x;
	target_pose.position.y = y;
	target_pose.position.z = z;

	move_group.setJointValueTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan plan;

	// bool success = false;
	// while (!success) {
	bool success = move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	// }

	ROS_INFO_NAMED("", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
	if (!success) return false;

	vector<double> toPush;

	vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points;
	trajectory_points = plan.trajectory_.joint_trajectory.points;
	trajectory_msgs::JointTrajectoryPoint endpoint = trajectory_points[trajectory_points.size() - 1];

	for (size_t i = 0; i < 6; ++i) {
		joint_group_positions[i] = endpoint.positions[i];
		if (i < 4) {
			toPush.push_back(joint_group_positions[i] * big_servo);
		} else {
			toPush.push_back(joint_group_positions[i] * sml_servo);
		}
		ROS_INFO("Joint %-25s: %f %f", joint_names[i].c_str(), joint_group_positions[i], toPush[i]);
	}

	toPush[0] += servos[0];
	toPush[2] = servos[2] + toPush[1] + toPush[2];
	toPush[1] = servos[1] - toPush[1];
	toPush[3] += servos[3];
	toPush[4] = servos[4] - toPush[4];
	toPush[5] = servos[5] - toPush[5];

	stringstream ss;

	for (size_t i = 0; i < toPush.size(); ++i) {
		ss << toPush[i] << ' ';
	}
	
	std_msgs::String msg;
	msg.data = ss.str();
	PWM_publisher.publish(msg);

	move_group.execute(plan);

	cout << "Raw Input: ";
	cout << roll << ' ' << pitch << ' ' << yaw << ' ';
	cout << x << ' ' << y << ' ' << z << '\n';

	cout << "Target Position: ";
	cout << roll << ' ' << pitch << ' ' << yaw << ' ';
	cout << target_pose.position.x << ' ';
	cout << target_pose.position.y << ' ';
	cout << target_pose.position.z << '\n';
	
	current_pose = move_group.getCurrentPose().pose;
	geometry_msgs::Point end_position = current_pose.position;
	geometry_msgs::Quaternion end_quaternion = current_pose.orientation;

	tf::Quaternion tfquaternion = tf::Quaternion(end_quaternion.x, end_quaternion.y, end_quaternion.z, end_quaternion.w);
	tfquaternion.normalize();
	double end_roll, end_pitch, end_yaw;
	tf::Matrix3x3(tfquaternion).getRPY(end_roll, end_pitch, end_yaw);

	cout << "End Position: ";
	cout << end_roll << ' ' << end_pitch << ' ' << end_yaw << ' ';
	cout << end_position.x << ' ' << end_position.y << ' ' << end_position.z << '\n';

	double x_diff = fabs(x - end_position.x);
	double y_diff = fabs(y - end_position.y);
	double z_diff = fabs(z - end_position.z);

	if (x_diff < moe && y_diff < moe && z_diff < moe) {
		cout << "SUCCESS" << '\n' << '\n';
		return true;
	} else {
		cout << "FAILED" << '\n' << '\n';
		return false;
	}
}

void publish_gripper(string s, int times) {
	std_msgs::String msg;
	msg.data = s;

	for (int i = 0; i < times; ++i) {
		PWM_publisher.publish(msg);
		// this_thread::sleep_for(chrono::milliseconds(1000));
		// usleep(1000000);
	}
}

bool isExecuting;

void execute(string input) {
	isExecuting = true;

	for (int i = 0; i < input.length(); i++) {
		if (input[i] == 'n') {
			isExecuting = false;
			return;
		}
	}
	
	try {
		cout << input << endl;

		istringstream iss(input);
		vector<string> tokens {
			istream_iterator<string>{iss},
			istream_iterator<string>{}
		};
		if (tokens.size() != 6) throw tokens.size();

		vector<double> d(6);
		for (int i = 0; i < 6; ++i) {
			d[i] = stod(tokens[i]);
		}
		double x1, y1, z1, x2, y2, z2, xm, ym;
		x1 = d[0];
		y1 = d[1];
		z1 = d[2];
		x2 = d[3];
		y2 = d[4];
		z2 = d[5];
		xm = (x1 + x2) / 2;
		ym = (y1 + y2) / 2;

		cout << x1 << ' ' << y1 << ' ' << z1 << ' ' << x2 << ' ' << y2 << ' ' << z2 << endl;

		move(0.0, 1.57, 0, x1, y1, 0.04);
		move(0.0, 1.57, 0, x1, y1, 0.0);
		publish_gripper("c", 1);
		usleep(2500000); // allow gripper to close completely
		move(0.0, 1.57, 0, x1, y1, 0.01);
		move(0.0, 1.57, 0, x1, y1, 0.02);
		// publish_gripper("c", 1);
		move(0.0, 1.57, 0, x1, y1, 0.05);
		// publish_gripper("c", 1);
		// move(0.0, 1.57, 0, x1, y1, 0.06);
		// move(0.0, 1.57, 0, xm, ym, 0.05);
		// publish_gripper("c", 1);
		

		move(0.0, 1.57, 0, x2, y2, 0.05);
		move(0.0, 1.57, 0, x2, y2, 0.03);
		publish_gripper("o", 2);

		move(0.0, 0.0, 0.0, home_x, home_y, home_z);
		// this_thread::sleep_for(chrono::seconds(5));
		usleep(3000000);


	} catch (const std::invalid_argument& invarg) {
		cout << "Invalid argument(s)." << endl;
	} catch (int numargs) {
		cout << "Incorrect number of arguments. Required: 6/13, Found: " << numargs << endl;
	} catch (unsigned long other) {
		cout << other << endl;
	}
	
	isExecuting = false;
}

void call_back(const std_msgs::String::ConstPtr& msg) {
	string input = msg->data.c_str();

	if (!isExecuting) {
		execute(input);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robit_move_group_interface");
	ros::NodeHandle PWM_node_handle;
	ros::NodeHandle corner_node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	init();

	PWM_publisher = PWM_node_handle.advertise<std_msgs::String>("PWMvalues", 1000);
	ros::Subscriber corner_subscriber = corner_node_handle.subscribe("corners", 1, call_back);

	publish_gripper("o", 1);
	while (true) {
		ros::spinOnce();
	}
	

	// for user input via command line
	
	/*
	bool flag = true;
	while (true) {
		string input;
		getline(cin, input);

		if (input.compare("exit") == 0) break;
		try {
			istringstream iss(input);
			vector<string> tokens{istream_iterator<string>{iss}, istream_iterator<string>{}};
			if (tokens.size() != 6) throw 1;
			vector<double> d(6);
			for (int i = 0; i < 6; i++) {
				d[i] = stod(tokens[i]);
			}
			move(d[0], d[1], d[2], d[3], d[4], d[5]);
			// std_msgs::String msg;
			// if (flag) {
			// 	msg.data = "c";
			// } else {
			// 	msg.data = "o";
			// }
			// flag = !flag;
		} catch (const std::invalid_argument& invarg) {
			cout << "Bad Argument(s)" << endl;
		} catch (int numargs) {
			bool bad = (numargs != 1) || (input.compare("c") != 0) && (input.compare("o") != 0);

			if (bad) {
				cout << input;
				cout << "Incorrect number of arguments. Required: 6, Found: " << numargs << endl;
			} else {
				std_msgs::String msg;
				msg.data = input;
				PWM_publisher.publish(msg);
				this_thread::sleep_for(chrono::milliseconds(1000));
			}

		}
	}
	*/


	ros::shutdown();
	return 0;
}
