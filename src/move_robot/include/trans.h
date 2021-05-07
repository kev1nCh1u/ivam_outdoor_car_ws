#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>


class trans
{
public:
	void robot2world(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y);
	void robot2world(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float obs_way_theta, float &x, float &y);
	
	void world2robot(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y);
	void world2robot(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float obs_way_theta, float &x, float &y);

	void closeline(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, float &m_error);

};

void trans::robot2world(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y)
{
	float x_world = target_pos.x()*cos(robot_pos.z() ) - target_pos.y()*sin(robot_pos.z() );
        float y_world = target_pos.x()*sin(robot_pos.z() ) + target_pos.y()*cos(robot_pos.z() );
	x = x_world + robot_pos.x();
	y = y_world + robot_pos.y();
}
void trans::robot2world(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float obs_way_theta, float &x, float &y)
{
	float x_world = target_pos.x()*cos(robot_pos.z() + obs_way_theta) - target_pos.y()*sin(robot_pos.z() + obs_way_theta);
        float y_world = target_pos.x()*sin(robot_pos.z() + obs_way_theta) + target_pos.y()*cos(robot_pos.z() + obs_way_theta);
	x = x_world + robot_pos.x();
	y = y_world + robot_pos.y();
}
void trans::world2robot(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &x, float &y)
{
	float x_error_world = target_pos.x() - robot_pos.x();
        float y_error_world = target_pos.y() - robot_pos.y();
        float x_error_robot = x_error_world*cos(-1*robot_pos.z()) - y_error_world*sin(-1*robot_pos.z()); 
        float y_error_robot = x_error_world*sin(-1*robot_pos.z()) + y_error_world*cos(-1*robot_pos.z());
	x = x_error_robot;
	y = y_error_robot;
}
void trans::world2robot(Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float obs_way_theta, float &x, float &y)
{
	float x_error_world = target_pos.x() - robot_pos.x();
        float y_error_world = target_pos.y() - robot_pos.y();
        float x_error_robot = x_error_world*cos(-1*robot_pos.z() - obs_way_theta) - y_error_world*sin(-1*robot_pos.z() - obs_way_theta);
        float y_error_robot = x_error_world*sin(-1*robot_pos.z() - obs_way_theta) + y_error_world*cos(-1*robot_pos.z() - obs_way_theta);	
	x = x_error_robot;
	y = y_error_robot;
}

//點到線的距離
void trans::closeline(Eigen::Vector3f finial_target_pose, Eigen::Vector3f target_pos, Eigen::Vector3f robot_pos, float &m_error)
{
	
	Eigen::Vector3f finial_target_p ;
        finial_target_p << finial_target_pose.head<2>(),1.0;
        Eigen::Vector3f target_p ;
        target_p << target_pos.head<2>(),1.0;
        Eigen::Vector3f robot_p ;
        robot_p << robot_pos.head<2>(),1.0;

        
        Eigen::Vector3f target_line;
        target_line=finial_target_p.cross(target_p);

	float scale = sqrt(target_line.x() * target_line.x() + target_line.y() * target_line.y());
	Eigen::Vector3f normalize_Line;
	normalize_Line << target_line.x()/scale,target_line.y()/scale,target_line.z()/scale;

        m_error = normalize_Line.transpose() * robot_p;

//std::cout<<"m_error  "<<m_error<<std::endl;

        
}
