

//ros
#include <ros/ros.h>
#include "ros/package.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//----------自創
#include <AnhungControl/Node_recv.h>
#include <AnhungControl/traffic_recv.h>
#include <AnhungControl/state.h>
#include <AnhungControl/joystick.h>
#include <AnhungControl/Battery.h>
#include <AnhungControl/setmap_ctr.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

//qt
#include <QByteArray>
#include <QBuffer>
#include <QImage>
#include <QPixmap>
#include <QDebug>

//IP
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>


//c++
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <vector>
#include <boost/thread.hpp>
#include <string>
#include <Eigen/Geometry>
#include <fstream>
#include <time.h>

#include <mysql/mysql.h>

#define CARID 0

#define Command_STATE_IDLE 0
#define Command_STATE_TRACKING 1
#define Command_STATE_Traffic 2
#define Command_STATE_Loading 3
#define Command_STATE_unLoading 4
#define Command_STATE_ChangeMode 5
#define Command_STATE_TemporaryMode 6
#define Command_STATE_FreeLoading 7
#define Command_STATE_virtual_Traffic 8
#define Command_Elevator_Entrance 9
#define Command_Elevator_Inside 10
#define Command_Samefloor_ChangeMap 11

#define PUSE_BUTTON_A 0
#define PUSE_BUTTON_B 1
#define PUSE_BUTTON_X 2
#define PUSE_BUTTON_Y 3
#define PUSE_BUTTON_RB 5
#define PUSE_BUTTON_START 7
#define CarParameterPATH_Local "/src/move_robot/parameter/car_parameter"
#define LocalparPATH_Local "/src/move_robot/parameter/local_parameter"
#define imagePATH_Local "/ros_map/ivam_1F_0.pgm"

#define ReloadCarParameter 0
#define ReloadLocalParameter 1

struct NODE_recv{

	int id;
	int type;
	std::string kin;
	float id_pixel_x;
	float id_pixel_y;
	float id_heading;
	int time;
	bool btn_finish;
	int line;
	float radius;
	std::string map;
	int floor;

	Eigen::Vector3f node_pose;
};

struct Local_par{
//local_parameter
float tracking_kp;
float tracking_kd;
int Car_packMode;
float ChangeMode_dis_error;
float ChangeMode_angular_error;
float traffic_dis_error;
float othermode_dis_error;
float othermode_angular_error;
float target_index_V_ratio;
float target_index_distance;
std::string network_card;
std::string map_name;
};

struct Car_par{
	//Car_parameter
	std::string CarName_;	//車子名稱
	int Carnumber;
    	int CarKind;		//車種()
    	float CarLength;
    	float CarWidth;
    	float LRWheeldis;
    	float FBWheeldis;
    	float LWheeldis;
    	float Wheel_r;
    	float Reduction_Ratio;
    	float speedup;
	float speed;
	//JOYSTICK
	float JOYSTICK_SCALAR;
	//OBS
	float OBS_Front;
	float OBS_Side;
	float OBS_Front_limit;
	float OBS_Stop_Sec;	//幾秒內停下
	float OBS_Wait_Sec;	//等待幾秒障礙物離開
	float OBS_Theta_Sec;
	float traffic_dis;
	float reMission_time;

};

struct traffic_recv{
	int id;
	int GO;
	float speed;

};

class Control{

public:

	Control(char *address, char *SRV_IP, int port);
	~Control();

	void ErrorCallback(const std_msgs::Int8& msg);
	void LoadCarParameter(std::string file_buf);
	void SetLocalpar(std::string file_buf);
	void RevProcess(double receive_period);
	void mapCallback(const nav_msgs::OccupancyGrid& map);
	void slamoutCallback(const geometry_msgs::PoseStamped& msg);
	void MissMsgCallback(const std_msgs::Int16& msg);
	void STATECallback(const AnhungControl::state& msg);
	void joystickCallback(const AnhungControl::joystick& joystick);
	void ReMissionStateCallback(const std_msgs::Int8& msg);
	void BatteryCallback(const AnhungControl::Battery& msg);
	void MapnameCallback(const std_msgs::String& msg);

	void ErrorState(int i);
	void vCallback(const std_msgs::Float32& msg);

	QPixmap Base64_To_Image(QByteArray bytearray);

	QByteArray Image_To_Base64(QImage &image);

	std::string int2str(int i);

	std::string float2str(float i);

	int recvtimeout(int ss, char *buf, int len, int timeout);

	void setMapTransformation(const Eigen::Vector2f &StartCoord, float cellLength);

	void setTime();
	void LoadTitlePath();
	void Opensend();
	void SearchIP();
	void sendScale();

    Eigen::Vector2f getWorldCoords(const Eigen::Vector2f& mapCoords);

    Eigen::Vector2f getMapCoords(const Eigen::Vector2f& worldCoords);

    Eigen::Vector3f getWorldCoordsPose(const Eigen::Vector3f& mapPose);

    Eigen::Vector3f getMapCoordsPose(const Eigen::Vector3f& worldPose);




public:
	bool isSocketOpen_;
	std::vector<NODE_recv> NodeSet;


private:

	ros::NodeHandle nodeHandle_;

	ros::Subscriber JoysickSubscriber_;
	ros::Subscriber vSubscriber_;
	ros::Subscriber ErrorSubscriber_;
	ros::Subscriber map_Subscriber_;
	ros::Subscriber MissMsgSubscriber_;
	ros::Subscriber STATESubscriber_;
	ros::Subscriber slamoutSubscriber_;
	ros::Subscriber ReMissionStateSubscriber_;
	ros::Subscriber BatterySubscriber_;
	ros::Subscriber MapnameSubscriber_;

	ros::Publisher Command_Publisher_;;
	ros::Publisher IdType_Publisher_;
	ros::Publisher trafficGO_Publisher_;
	ros::Publisher  Clear_ObsMode_Publisher_;
	ros::Publisher Path_Publisher_;
	ros::Publisher Trigger_Publisher_;
	ros::Publisher floor_Publisher_;

	boost::thread* recv_command_thread_;

	struct sockaddr_in si_other_, si_me_, si_BROADCAST;
	char *IP;
	int Port;

	int sfd_, slen_, clientlen;

	char *argv;

	std::vector<int> path;

	bool isSetMapTrans;

	float map_resloution;
	float map_width_size, map_height_size;
	std::string Mapname;

	Eigen::Affine2f mapTworld, worldTmap;
	Eigen::Vector3f recv_map;

	bool isLoadMap;

	int max_map_width;
	int max_map_height;
	float m_resolution;

	traffic_recv traffic_recv_buf;

	int isMissing;
	int Command_STATE;
	int Command_id;

	int btn_id;
	bool isReveice_joystick;

	Car_par   par;
	Local_par Lc_par;

	int commmand_buf;
	std::string Name_buf;

	float V_now ;
	bool SendNode;
	bool protect_SendNode;

	//PATH
	std::string TitlePath;
	std::string CarParameterPATH;
	std::string LocalparPATH;
	std::string imagePATH;

	//ConnectTest
	std::string Connect_buf;
	int registration;

	//SearchIP
	std::string IP_buf;

	//Confirm floor
	int Floor_buf;

	//Car globle data
	std::string CarName;
	std::string Carnumber;
	std::string MAP_NAME ;

	//reMission
	int Start_ID;
	int End_ID;
	float End_x;
	float End_y;
	float End_z;
	Eigen::Vector3f robot_pose_;

	//battery
	float Voltage;
	float Current;
	float RelativeSOC1;
	float RelativeSOC2;
	float RelativeSOC3;
	float RelativeSOC4;
	float AbsoluteSOC1;
	float AbsoluteSOC2;
	float AbsoluteSOC3;
	float AbsoluteSOC4;
	float Temp1;
	float Temp2;
	float Temp3;
	float Temp4;

	//Time
	std::string Time_date;
	std::string Time_;
	ros::Time begin_time;
	ros::Time mission_begin_time;
	int last_all_sec;
	int last_all_missiontime;
	int all_missiontime ;
	int missiontime;
	std::string time_path;

};
