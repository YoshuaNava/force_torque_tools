/*
 *  ft_calib_node.cpp
 *
 *
 *  Created on: Sep 26, 2012
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2012, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <random>
#include <sstream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <force_torque_sensor_calib/ft_calib.h>
#include <Eigen/Core>

using namespace Calibration;


class FTCalibNode
{


public:
	ros::NodeHandle nh_;
	ros::AsyncSpinner *spinner;
	ros::Subscriber topicSub_ft_raw_;
	ros::Subscriber topicSub_Accelerometer_;

	FTCalibNode()
	{
		nh_ = ros::NodeHandle("~");
		spinner = new ros::AsyncSpinner(1);
		spinner->start();



		topicSub_ft_raw_ = nh_.subscribe("ft_raw", 1, &FTCalibNode::topicCallback_ft_raw, this);
		topicSub_Accelerometer_ = nh_.subscribe("imu", 1, &FTCalibNode::topicCallback_imu, this);

		m_pose_counter = 0;
		m_ft_counter = 0;

		m_received_ft = false;
		m_received_imu = false;

		m_finished = false;
		m_tf_listener = new tf::TransformListener();

		m_ft_calib = new FTCalib();
	}

	~FTCalibNode()
	{
		saveCalibData();
		delete spinner;
		delete m_group;
		delete m_ft_calib;
		delete m_tf_listener;
	}


	bool getROSParameters()
	{
		// Get the moveit group name
		if(nh_.hasParam("moveit_group_name"))
		{
			nh_.getParam("moveit_group_name", m_moveit_group_name);
		}

		else
		{
			ROS_ERROR("No moveit_group_name parameter, shutting down node...");
			nh_.shutdown();
			return false;
		}

		// Get the name of output calibration file
		if(nh_.hasParam("calib_file_name"))
		{
			nh_.getParam("calib_file_name", m_calib_file_name);
		}

		else
		{
			ROS_WARN("No calib_file_name parameter, setting to default 'ft_calib.yaml'");
			m_calib_file_name = std::string("ft_calib_data.yaml");
		}

        // Get the name of calibration file directory
        if(nh_.hasParam("calib_file_dir"))
        {
            nh_.getParam("calib_file_dir", m_calib_file_dir);
        }

        else
        {
            ROS_WARN("No calib_file_dir parameter, setting to default '~/.ros/ft_calib' ");
            m_calib_file_dir = std::string("~/.ros/ft_calib");
        }


        // Get the name of file to store the gravity and F/T measurements
        if(nh_.hasParam("meas_file_name"))
        {
            nh_.getParam("meas_file_name", m_meas_file_name);
        }

        else
        {
            ROS_WARN("No meas_file_name parameter, setting to default 'ft_calib_meas.txt'");
            m_meas_file_name = std::string("ft_calib_meas.txt");
        }

        // Get the name of directory to save gravity and force-torque measurements
        if(nh_.hasParam("meas_file_dir"))
		{
            nh_.getParam("meas_file_dir", m_meas_file_dir);
		}

		else
		{
            ROS_WARN("No meas_file_dir parameter, setting to default '~/.ros/ft_calib' ");
            m_meas_file_dir = std::string("~/.ros/ft_calib");
		}

		if(nh_.hasParam("ft_sensor_frame_id"))
		{
			nh_.getParam("ft_sensor_frame_id", m_ft_sensor_frame_id);
		}
		else
        {
            ROS_ERROR("No ft_sensor_frame_id parameter, shutting down node ...");
            nh_.shutdown();
            return false;
        }

		if(nh_.hasParam("poses_frame_id"))
		{
			nh_.getParam("poses_frame_id", m_poses_frame_id);
		}
        else
        {
            ROS_ERROR("No poses_frame_id parameter, shutting down node ...");
            nh_.shutdown();
            return false;
        }

		if (nh_.hasParam("cone_angle"))
		{
			nh_.getParam("cone_angle", m_cone_angle);
		}
		else
		{
			m_cone_angle = 0.5;
			ROS_WARN("Missing cone_angle parameter, using default value of %f radians", m_cone_angle);
		}

		
		// operation mode
		nh_.param("operation_mode", op_mode, MANUAL_POSITIONING);

        // number of random poses
        nh_.param("number_poses", m_number_poses, 30);


        // initialize the file with gravity and F/T measurements

        // expand the path
        if (!m_meas_file_dir.empty() && m_meas_file_dir[0] == '~') {
            assert(m_meas_file_dir.size() == 1 or m_meas_file_dir[1] == '/');  // or other error handling
            char const* home = getenv("HOME");
            if (home or (home = getenv("USERPROFILE"))) {
                m_meas_file_dir.replace(0, 1, home);
            }
            else {
                char const *hdrive = getenv("HOMEDRIVE"),
                        *hm_meas_file_dir = getenv("HOMEPATH");
                assert(hdrive);  // or other error handling
                assert(hm_meas_file_dir);
                m_meas_file_dir.replace(0, 1, std::string(hdrive) + hm_meas_file_dir);
            }
        }

        std::ofstream meas_file;
        meas_file.open((m_meas_file_dir + "/" + m_meas_file_name).c_str(), std::ios::out);

        std::stringstream meas_file_header;

        meas_file_header << "\% gravity , f/t measurements all expressed in F/T sensor frame\n";
        meas_file_header << "\% [gx, gy, gz, fx, fy, fz, tx, ty, tz]\n";

        meas_file << meas_file_header.str();

        meas_file.close();

        return true;
    }
    // connects to the move arm servers
	void initMoveGroupInterface()
	{
		if(op_mode != MANUAL_POSITIONING)
			m_group = new moveit::planning_interface::MoveGroupInterface(m_moveit_group_name);
	}


	// Calibrates the FT sensor by putting the arm in several different positions
	bool moveNextPose()
	{

		std::stringstream ss;
		ss << m_pose_counter;
		Eigen::Matrix<double, 6, 1> pose;

		if(op_mode == PREDEFINED_POSES)
		{
			// find poses from the parameter server
			// poses should be in "pose%d" format (e.g. pose0, pose1, pose2 ...)
			// and they should be float arrays of size 6
			ROS_INFO("PREDEFINED_POSE");
			if(!getPose("pose"+ss.str(), pose))
			{
				ROS_INFO("Finished group %s poses", m_group->getName().c_str());
				m_finished = true;
				return true;
			}

			geometry_msgs::Pose pose_;
			pose_.position.x = pose(0);
			pose_.position.y = pose(1);
			pose_.position.z = pose(2);

			tf::Quaternion q;
			q.setRPY((double)pose(3), (double)pose(4), (double)pose(5));

			tf::quaternionTFToMsg(q, pose_.orientation);

			geometry_msgs::PoseStamped pose_stamped;
			pose_stamped.pose = pose_;
			pose_stamped.header.frame_id = m_poses_frame_id;
			pose_stamped.header.stamp = ros::Time(0);

			m_group->setPoseTarget(pose_stamped);

			ROS_INFO("Executing pose %d", m_pose_counter);
			ROS_INFO("EE goal position = (%f, %f, %f)", pose(0), pose(1), pose(2));
			ROS_INFO("EE goal orientation = (%f, %f, %f, %f)", q.x(), q.y(), q.z(), q.w());

			m_group->move();
		}
		else if(op_mode == RANDOM_POSES) // or execute random poses
		{
			// generate poses inside a cone, with a pre-defined angle with respect to its main axis (m_cone_angle)
			ROS_INFO("RANDOM_POSE");
			if(m_pose_counter < m_number_poses)
			{
				if(!getPose("pose_base", pose))
				{
					ROS_INFO("Base pose not found for move group %s", m_group->getName().c_str());
					m_finished = true;
					return false;
				}

				// srand(time(NULL));
				geometry_msgs::Pose pose_;
				pose_.position.x = pose(0);
				pose_.position.y = pose(1);
				pose_.position.z = pose(2);

				tf::Quaternion q;
				std::random_device rd;  //Will be used to obtain a seed for the random number engine
				std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
				std::uniform_real_distribution<> angle_dist(0, 2*M_PI), coord_dist(cos(m_cone_angle), 1), rot_dist(0, 2*M_PI);
				tfScalar coord, phi, rot;

				coord = coord_dist(gen);
				phi = angle_dist(gen);
				rot = rot_dist(gen);

				geometry_msgs::PoseStamped current_pose = m_group->getCurrentPose();
				tf::Vector3 direction(coord, sqrt(1 - coord*coord)*cos(phi), sqrt(1 - coord*coord)*sin(phi)); // Direction we want to point to
				tf::Vector3 base_direction(0, 0, 1); // Base
				tf::Vector3 diff_v = base_direction.cross(direction);
				tfScalar diff_rot = 1 + base_direction.dot(direction);
				tf::Quaternion q_init(diff_v.x(), diff_v.y(), diff_v.z(), diff_rot);
				tf::Quaternion q_post(0, 0, 1, rot);
				q_post.normalize();
				q_init.normalize();

				q.setRotation(direction, 0);
				q.normalize();
				q = q_init*q*q_post;
	
				tf::quaternionTFToMsg(q, pose_.orientation);
	
				geometry_msgs::PoseStamped pose_stamped;
				pose_stamped.pose = pose_;
				pose_stamped.header.frame_id = m_poses_frame_id;
				pose_stamped.header.stamp = ros::Time(0);
	
				m_group->setPoseTarget(pose_stamped);

				ROS_INFO("Executing pose %d", m_pose_counter);
				ROS_INFO("EE goal position = (%f, %f, %f)", pose(0), pose(1), pose(2));
				ROS_INFO("EE goal orientation = (%f, %f, %f, %f)", q.x(), q.y(), q.z(), q.w());

				m_group->move();
			}
			else
			{	
					ROS_INFO("Finished group %s random poses", m_group->getName().c_str());
					m_finished = true;
				
			}
		}
		else
		{
			if(m_pose_counter < m_number_poses)
			{
				ROS_INFO("MANUAL_POSITIONING");
				ROS_INFO("Pose %d", m_pose_counter);
				ROS_INFO("Position the end effector in the desired pose and press ENTER to continue.");
				while(ros::ok() && getchar()!='\n')
				{ }
				ROS_INFO("Pose captured. Please keep the end effector static until you're asked to reposition it again.");
			}
			else
			{
				ROS_INFO("Finished group %s manually set poses", m_group->getName().c_str());
				m_finished = true;
			}
		}

		m_pose_counter++;
		
		ROS_INFO("Finished executing pose %d", m_pose_counter-1);
		return true;
	}

	// gets the next pose from the parameter server
	// pose in [x y z r p y] format ([m], [rad])
	bool getPose(const std::string &pose_param_name, Eigen::Matrix<double, 6, 1> &pose)
	{
		XmlRpc::XmlRpcValue PoseXmlRpc;
		if(nh_.hasParam(pose_param_name))
		{
			nh_.getParam(pose_param_name, PoseXmlRpc);
		}

		else
		{
			ROS_WARN("Pose parameter %s not found", pose_param_name.c_str());
			return false;
		}

		if(PoseXmlRpc.size()!=6)
		{
			ROS_ERROR("Pose parameter %s wrong size (must be 6)", pose_param_name.c_str());
			return false;
		}

		for(unsigned int i=0; i<6; i++)
			pose(i) = (double)PoseXmlRpc[i];

		return true;
	}

	// prints out the pose (3-D positions) of the calibration frame at each of the positions
	// of the left arm
	void saveCalibData()
	{
		double mass;
		Eigen::Vector3d COM_pos;
		Eigen::Vector3d f_bias;
		Eigen::Vector3d t_bias;

		getCalib(mass, COM_pos, f_bias, t_bias);

		XmlRpc::XmlRpcValue bias;
		bias.setSize(6);
		for(unsigned int i=0; i<3; i++)
			bias[i] = (double)f_bias(i);

		for(unsigned int i=0; i<3; i++)
			bias[i+3] = (double)t_bias(i);

		XmlRpc::XmlRpcValue COM_pose;
		COM_pose.setSize(6);
		for(unsigned int i=0; i<3; i++)
			COM_pose[i] = (double)COM_pos(i);

		for(unsigned int i=0; i<3; i++)
			COM_pose[i+3] = 0.0;

		// set the parameters in the parameter server
		nh_.setParam("/ft_calib/bias", bias);
		nh_.setParam("/ft_calib/gripper_mass", mass);
		nh_.setParam("/ft_calib/gripper_com_frame_id", m_ft_raw.header.frame_id.c_str());
		nh_.setParam("/ft_calib/gripper_com_pose", COM_pose);

		// dump the parameters to YAML file
		std::string file = m_calib_file_dir + std::string("/") + m_calib_file_name;

		// first create the directory
		std::string command = std::string("mkdir -p ") + m_calib_file_dir;
		std::system(command.c_str());

		// now dump the yaml file
		command.clear();
		command = std::string("rosparam dump ") + file + std::string(" /ft_calib");
		std::system(command.c_str());
	}

    // saves the gravity and force-torque measurements to a file for postprocessing
    void saveMeasurements(geometry_msgs::Vector3Stamped gravity, geometry_msgs::WrenchStamped ft_meas)
    {
        std::ofstream meas_file;
        meas_file.open((m_meas_file_dir + "/" + m_meas_file_name).c_str(), std::ios::out | std::ios::app);

        std::stringstream meas_file_text;

        meas_file_text << gravity.vector.x << " " << gravity.vector.y << " " << gravity.vector.z << " ";
        meas_file_text << ft_meas.wrench.force.x << " " << ft_meas.wrench.force.y << " " << ft_meas.wrench.force.z << " ";
        meas_file_text << ft_meas.wrench.torque.x << " " << ft_meas.wrench.torque.y << " " << ft_meas.wrench.torque.z << "\n";

        meas_file << meas_file_text.str();

        meas_file.close();
    }

	// finished moving the arm through the poses set in the config file
	bool finished()
	{
		return(m_finished);
	}

	void topicCallback_ft_raw(const geometry_msgs::WrenchStamped::ConstPtr &msg)
	{
		ROS_DEBUG("In ft sensorcallback");
		m_ft_raw = *msg;

		// check if msg has a frame id
		if (m_ft_raw.header.frame_id.empty())
		{
			m_ft_raw.header.frame_id = m_ft_sensor_frame_id;
		}

		m_received_ft = true;
	}


	// gets readings from accelerometer and transforms them to the FT sensor frame
	void topicCallback_imu(const sensor_msgs::Imu::ConstPtr &msg)
	{
		ROS_DEBUG("In accelerometer read callback");

		m_imu= *msg;
		m_received_imu = true;
	}

	void addMeasurement()
	{

		m_ft_avg.wrench.force.x = -m_ft_avg.wrench.force.x/(double)m_ft_counter;
		m_ft_avg.wrench.force.y = -m_ft_avg.wrench.force.y/(double)m_ft_counter;
		m_ft_avg.wrench.force.z = -m_ft_avg.wrench.force.z/(double)m_ft_counter;

		m_ft_avg.wrench.torque.x = -m_ft_avg.wrench.torque.x/(double)m_ft_counter;
		m_ft_avg.wrench.torque.y = -m_ft_avg.wrench.torque.y/(double)m_ft_counter;
		m_ft_avg.wrench.torque.z = -m_ft_avg.wrench.torque.z/(double)m_ft_counter;


		m_ft_counter = 0;

		if(!m_received_ft)
		{
			ROS_ERROR("Haven't received F/T sensor measurements");
			return;
		}

		if(!m_received_imu)
		{
			ROS_ERROR("Haven't received accelerometer readings");
			return;
		}

		// express gravity vector in F/T sensor frame
		geometry_msgs::Vector3Stamped gravity;
		gravity.header.stamp = ros::Time();
		gravity.header.frame_id = m_imu.header.frame_id;
		gravity.vector = m_imu.linear_acceleration;

		geometry_msgs::Vector3Stamped gravity_ft_frame;

		try
		{
			m_tf_listener->transformVector(m_ft_raw.header.frame_id, gravity, gravity_ft_frame);
		}

		catch(tf::TransformException &ex)
		{
			ROS_ERROR("Error transforming accelerometer reading to the F/T sensor frame");
			ROS_ERROR("%s.", ex.what());
			return;
		}

		m_ft_calib->addMeasurement(gravity_ft_frame, m_ft_avg);
        saveMeasurements(gravity_ft_frame, m_ft_avg);
	}

	void getCalib(double &mass, Eigen::Vector3d &COM_pos, Eigen::Vector3d &f_bias, Eigen::Vector3d &t_bias)
	{

		Eigen::VectorXd ft_calib = m_ft_calib->getCalib();

		mass = ft_calib(0);
		if(mass<=0.0)
		{
			ROS_ERROR("Error in estimated mass (<= 0)");
			//		return;
		}

		Eigen::Vector3d center_mass_position(ft_calib(1)/mass,
				ft_calib(2)/mass,
				ft_calib(3)/mass);

		COM_pos = center_mass_position;

		f_bias(0) = -ft_calib(4);
		f_bias(1) = -ft_calib(5);
		f_bias(2) = -ft_calib(6);
		t_bias(0) = -ft_calib(7);
		t_bias(1) = -ft_calib(8);
		t_bias(2) = -ft_calib(9);

	}

	void averageFTMeas()
	{

		if(m_ft_counter==0)
		{

			m_ft_avg = m_ft_raw;

		}

		else
		{

			m_ft_avg.wrench.force.x = m_ft_avg.wrench.force.x + m_ft_raw.wrench.force.x;
			m_ft_avg.wrench.force.y = m_ft_avg.wrench.force.y + m_ft_raw.wrench.force.y;
			m_ft_avg.wrench.force.z = m_ft_avg.wrench.force.z + m_ft_raw.wrench.force.z;

			m_ft_avg.wrench.torque.x = m_ft_avg.wrench.torque.x + m_ft_raw.wrench.torque.x;
			m_ft_avg.wrench.torque.y = m_ft_avg.wrench.torque.y + m_ft_raw.wrench.torque.y;
			m_ft_avg.wrench.torque.z = m_ft_avg.wrench.torque.z + m_ft_raw.wrench.torque.z;

		}
		m_ft_counter++;
	}


private:

	moveit::planning_interface::MoveGroupInterface *m_group;

	unsigned int m_pose_counter;
	unsigned int m_ft_counter;

	bool m_finished;

	bool m_received_ft;
	bool m_received_imu;

	// cone angle limits
	double m_cone_angle;

	// ft calib stuff
	FTCalib *m_ft_calib;

	// expressed in FT sensor frame
	geometry_msgs::WrenchStamped m_ft_raw;
	geometry_msgs::WrenchStamped m_ft_avg; // average over 100 measurements

	// accelerometer readings
	sensor_msgs::Imu m_imu;

	tf::TransformListener *m_tf_listener;

	//	***** ROS parameters ***** //
	// name of the moveit group
	std::string m_moveit_group_name;

	// name of output calibration file
	std::string m_calib_file_name;

	// name of output directory
	// default: ~/.ros/ft_calib
	std::string m_calib_file_dir;

    // name of file with recorded gravity and F/T measurements
    std::string m_meas_file_name;

    // name of directory for saving gravity and F/T measurements
    // default: ~/.ros/ft_calib
    std::string m_meas_file_dir;

	// frame id of the poses to be executed
	std::string m_poses_frame_id;

	// frame of the force torque measurements
	std::string m_ft_sensor_frame_id;

	// operation modes
	const int PREDEFINED_POSES = 1;
	const int RANDOM_POSES = 2;
	const int MANUAL_POSITIONING = 3;

	// operation mode choice. Either
	int op_mode;

	// number of random poses
	// default: 30
	int m_number_poses;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ft_calib_node");
	ros::NodeHandle nh;

	FTCalibNode ft_calib_node;
	if(!ft_calib_node.getROSParameters())
	{
		ft_calib_node.nh_.shutdown();
		ROS_ERROR("Error getting ROS parameters");

	}
	ft_calib_node.initMoveGroupInterface();

	/// main loop
	double loop_rate_;
	ft_calib_node.nh_.param("loop_rate", loop_rate_, 650.0);
	ros::Rate loop_rate(loop_rate_); // Hz

	// waiting time after end of each pose to take F/T measurements
	double wait_time;
	ft_calib_node.nh_.param("wait_time", wait_time, 4.0);

	bool ret = false;
	unsigned int nh_measurements = 0;

	ros::Time t_end_move_arm = ros::Time(0);

	while (ft_calib_node.nh_.ok() && !ft_calib_node.finished())
	{

		// Move the arm, then calibrate sensor
		if(!ret)
		{
			ret = ft_calib_node.moveNextPose();
			t_end_move_arm = ros::Time(0);
			sleep(wait_time);
		}

		// average 100 measurements to calibrate the sensor in each position
		else if ((ros::Time::now() - t_end_move_arm).toSec() > wait_time)
		{
			nh_measurements++;
			ft_calib_node.averageFTMeas(); // average over 100 measurements;

			if(nh_measurements==100)
			{
				ret = false;
				nh_measurements = 0;

				ft_calib_node.addMeasurement(); // stacks up measurement matrices and FT measurementsa
				double mass;
				Eigen::Vector3d COM_pos;
				Eigen::Vector3d f_bias;
				Eigen::Vector3d t_bias;

				ft_calib_node.getCalib(mass, COM_pos, f_bias, t_bias);
				std::cout << "-------------------------------------------------------------" << std::endl;
				std::cout << "Current calibration estimate:" << std::endl;
				std::cout << std::endl << std::endl;

				std::cout << "Mass: " << mass << std::endl << std::endl;

				std::cout << "Center of mass position (relative to FT sensor frame):" << std::endl;
				std::cout << "[" << COM_pos(0) << ", " << COM_pos(1) << ", " << COM_pos(2) << "]";
				std::cout << std::endl << std::endl;


				std::cout << "FT bias: " << std::endl;
				std::cout << "[" << f_bias(0) << ", " << f_bias(1) << ", " << f_bias(2) << ", ";
				std::cout << t_bias(0) << ", " << t_bias(1) << ", " << t_bias(2) << "]";
				std::cout << std::endl << std::endl;


				std::cout << "-------------------------------------------------------------" << std::endl << std::endl << std::endl;
				ft_calib_node.saveCalibData();
			}

		}


		ros::spinOnce();
		loop_rate.sleep();
	}

	ft_calib_node.saveCalibData();
	ros::shutdown();
	return 0;
}
