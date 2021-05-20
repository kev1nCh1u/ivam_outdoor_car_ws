
#include <string>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <bitset>
#include <vector>
#include <boost/thread.hpp>
#include <time.h>
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <cstdlib>
#include <algorithm>

#include <ros/ros.h>
#include "ros/package.h"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>


#include <move_robot/joystick.h>
#include <move_robot/Node_recv.h>
#include <move_robot/traffic_recv.h>
#include <move_robot/state.h>
#include <move_robot/setmap_ctr.h>
#include "trans.h"
#include "kinematics.h"
#include "sendrev.h"
//#include "MPC.h"





#define CarParameterPATH_Local "/move_robot/parameter/car_parameter"
#define LocalparPATH_Local "/move_robot/parameter/local_parameter"

#define PUSE_BUTTON_A 0
#define PUSE_BUTTON_B 1
#define PUSE_BUTTON_X 2
#define PUSE_BUTTON_Y 3
#define PUSE_BUTTON_RB 5
#define PUSE_BUTTON_BACK 6
#define PUSE_BUTTON_START 7



#define P_STATE_IDLE 0
#define P_STATE_MISSON 1
#define P_STATE_MOVE 2
#define P_STATE_STOP 3
#define P_STATE_TEMPORARY 4
#define P_STATE_AVOID_WAIT 5
#define P_STATE_AVOID_REPLANNING 6
#define P_STATE_AVOID_OBS 7

#define P_STATE_AVOID_DEADLOCK 9
#define P_STATE_TIME_DELAY 10


#define P_OBS_NORMAL 0
#define P_OBS_WAIT 1
#define P_OBS_REPLANNING 2
#define P_OBS_AVOID 3
#define P_OBS_DEADLOCK 4


#define OBS_Decay_V 0.1//避障減速參數
#define OBS_LookGap 8.0//gap觀看距離
#define OBS_SideMiss_Scale 1.3 //障礙物消失距離的倍率 （須大於1.25）
#define OBS_isObs 15;//幾個點視為障礙物

#define MISSON_traffic 2
#define MISSON_Loading_Time 3
#define MISSON_Loading_Interrupt 4
#define MISSON_unLoading_Time 5
#define MISSON_unLoading_Interrupt 6
#define MISSON_ChangeMode 7
#define MISSON_TemporaryMode 8
#define MISSON_FreeLoading 9
#define MISSON_Virtual_traffic 10
#define MISSON_Elevator_Entrance 11
#define MISSON_Elevator_Inside 12
#define MISSON_Samefloor_ChangeMap 13


#define Command_STATE_IDLE 0
#define Command_STATE_TRACKING 1
#define Command_STATE_Traffic 2
#define Command_STATE_Loading 3
#define Command_STATE_unLoading 4
#define Command_STATE_ChangeMode 5
#define Command_STATE_TemporaryMode 6
#define Command_STATE_FreeLoading 7
#define Command_STATE_VirtualTraffic 8
#define Command_Elevator_Entrance 9
#define Command_Elevator_Inside 10
#define Command_Samefloor_ChangeMap 11

#define Clear_Mode 1
#define Close_AllObs 2
#define limitObs_Mode 3
#define Normal_Mode 4
#define Temporary_Mode 5

#define ReloadCarParameter 0

#define TrackingLineMarker 0
#define AvoidLineMarker 1
#define ALLOBSPointMarker 2
#define OBSPointMarker 3
#define obs_line_pointMarker 4
#define predict_line_pointMarker 5

#define obs_LineMarker 0
#define vertical_lineMarker 1

#define Min_nav_speed 0.25
#define reMission_time_freq 2


//全域變數
void TriggerCallback(const std_msgs::Int8& msg);
bool LoadCarKind(std::string file_buf,int& returnbuf);
bool mission_dont_push = false;
//LoadTitlePath
void LoadTitlePath();
void kind_search(int Carkind, char **argv, int baudrate);

std::string CarParameterPATH;
std::string TitlePath;
bool ReloadCarKind = false;
int baudrate;
char **argv_buf;
//PATH



struct NODE_recv{				//Jeff add
	int id;
	int type;
	std::string kin;
	int time;
	bool btn_finish;
	int line;
    float radius;
    std::string map;
    int floor;

	Eigen::Vector3f node_pose;
};

struct SUB_MISSONPATH_SUBPOINT{	//Jeff add
	int start;
	int start_type;
	int end_type;
	int end;
    std::string map;
    int floor;
	std::vector<Eigen::Vector3f> sub_missonPath_subPoint;
};


struct SUB_MISSONPATH{			//Jeff add

	//哪個點到哪個點(每條線) 對應之貝茲subPoint

	std::vector<SUB_MISSONPATH_SUBPOINT> sub_missonPath;
	std::vector<NODE_recv> ALL_pathnode;
};



struct trafficGO{				//Jeff add
	int id;
	int GO;
    float speed;
};

struct SERIAL{
    struct termios  oldtio;
    int fd,res,done;
    bool serial_ok;
};

struct SUBPATH{
    std::vector<Eigen::Vector3f> subpath;
};

struct ALLSUBPATH{
   std::vector<SUBPATH> mysubpath;
};

struct NODE{
    int id;
    std::string NodeType;
    Eigen::Vector3f World_coordinate;
    //std::vector<NEIGHBOR> neigbor;
};
struct gap
{
	Eigen::Vector3f p1;
	Eigen::Vector3f p2;
	float distance;
};



class Move_Robot{

public:

	Move_Robot(char *dev_name, int Baudrate);
	~Move_Robot();


 	//setup seril &file
 	int setBaudrate(int baudrate);
 	int SerialConnect(SERIAL &serial, char *port_name, int speed);
	//void LoadTitlePath();
	void LoadCarParameter(std::string file_buf);

	//tracking
	void Misson_Path(SUB_MISSONPATH &sub_missonPath_buf);
	void CreatPath(std::vector<Eigen::Vector3f> input_buf,std::vector<int> &id_Point_lineMode,std::vector<float> radius, SUB_MISSONPATH_SUBPOINT &return_buf);
	float getDistance(float x1, float y1, float x2, float y2);
	float FindAngle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);
	void CalculateCurve(std::vector<Eigen::Vector3f> &input_buf, SUB_MISSONPATH_SUBPOINT &return_buf, float step);
	//bool Navigation_move(std::vector<SUB_MISSONPATH_SUBPOINT> & myAllSubPath, bool isReSet);
	// bool Tracking_Angle_Init(int &subpath_index, bool isReSet);
	// bool Tracking_Trajectory(int &subpath_index, bool isReSet);
	// bool Trajectory_Tracking(int &subpath_index, bool isReSet);


	//other
	void NormalizeAngle(float& phi);
    void SendPackage(std::vector<unsigned char> recv);
	//void SendPackage(int FL, int FLA,int BL,int BLA,int FR ,int FRA,int BR,int BRA);
    void SendPackage_two(double cmd_vl, double cmd_vr);

	//state
	//void State_Machine(int& state);
	//void Stop();
	//void Idle();
	void Misson_state(bool isReSet);

	//Calculate
	//void Calculate_odom();
	int calc_target_index(Eigen::Vector3f &robot_pose, float &robot_v, std::vector<Eigen::Vector3f> &subpath, int &now_index);
	int calc_target_index_obs(Eigen::Vector3f &robot_pose, float &robot_v, std::vector<Eigen::Vector3f> &subpath, int &now_index);
	int calc_target_index_obsturn(Eigen::Vector3f &robot_pose, float &robot_v, std::vector<Eigen::Vector3f> &subpath, int &now_index);
    int calc_target_index_key(Eigen::Vector3f &robot_pose, float &dis, std::vector<Eigen::Vector3f> &subpath, int &now_index);
    int calc_target_index_MPC(Eigen::Vector3f &robot_pose,  std::vector<Eigen::Vector3f> &subpath, int &now_index);

	//joystick
	//void joystick_move();
	//void joystick_omni_move();

	//Laser
	int checkDeviceNum(const sensor_msgs::LaserScan& scan);
	tf::TransformListener tf_;
	std::string p_base_frame_;
	sensor_msgs::PointCloud laser_point_cloud_1_;
    sensor_msgs::PointCloud laser_point_cloud_2_;
	laser_geometry::LaserProjection projector_1_;
    laser_geometry::LaserProjection projector_2_;

	//ROS Subscriber & Publisher
	//void joystickCallback(const move_robot::joystick& joystick);
	void slamoutCallback(const geometry_msgs::PoseStamped& msg);
	void UKFCallback(const geometry_msgs::PoseStamped& msg);
	//void laserCallback(const sensor_msgs::LaserScan& scan);
	void IdTypeCallback(const move_robot::Node_recv& command);
	void TrafficGOCallback(const move_robot::traffic_recv& command);
	//void ClearCallback(const std_msgs::Int8& msg);
	void MissMsgCallback(const std_msgs::Int16& msg);
	void TriggerCallback_(const std_msgs::Int8& msg);
    void floorCallback_(const std_msgs::Int8& msg);



	// //Timer & Thread & SERIAL
	// //void timerCallback(const ros::TimerEvent& event);

    void draw(int id,float color_r,float color_g,float color_b,std::vector<Eigen::Vector3f>);
    void drawLine(int id,float color_r,float color_g,float color_b,std::vector<Eigen::Vector3f> drawline);
    int SetLocalpar(std::string file_buf);

    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order);

protected:
	Car_Kinematics Car;
	trans tra;
    sendrev sendreceive;
    //MPC mpc;

	//ROS Subscriber & Publisher
	ros::NodeHandle node_;
	ros::Publisher marker_Publisher_;
	ros::Publisher odometry_Publisher_;
	ros::Publisher Error_Publisher_;
	ros::Publisher v_Publisher_;
	ros::Publisher STATE_Publisher_;
	ros::Publisher ReMissionState_Publisher_;
  ros::Publisher marke_Publisher_ ;
  ros::Publisher Command_Publisher_;
	ros::Publisher Speak_Publisher_;

	ros::Subscriber JoysickSubscriber_;
	ros::Subscriber slamoutSubscriber_;
	ros::Subscriber ukfSubscriber_;
	ros::Subscriber laserSubscriber_;
	ros::Subscriber IdTypeSubscriber_;
	ros::Subscriber TrafficGOSubscriber_;
//	ros::Subscriber Clear_ObsModeSubscriber_;
	ros::Subscriber MissMsgSubscriber_;
	ros::Subscriber TriggerSubscriber_;
    ros::Subscriber floorSubscriber_;




	//joystick
	float joystick_theta;
	float joystick_v;

	std::string p_joystick_topic_;
	int p_joystick_subscriber_queue_size_;
	int btn_id;
	bool isReveice_joystick;


	//TImer sampling time
    	float time_sample;

	//Timer & Thread & SERIAL
	SERIAL mySerial;

	ros::Timer control_timer;
	boost::thread* receive_thread_;
    boost::thread* receive_Battery_thread_;




	//Car_parameter
	std::string CarName;	//車子名稱
	int Carnumber;
    int	CarKind;
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
	float OBS_Stop_Sec;
	float OBS_Wait_Sec;
	float OBS_Theta_Sec;

	float CarLength_helf ;
    float CarWidth_helf ;
    float LRWheeldis_helf ;
    float FBWheeldis_helf ;
	float traffic_dis;
	float reMission_time;

    float Car_Side;


    //local_parameter
    float tracking_kp;
    float tracking_kd;
		float ChangeMode_dis_error;
		float ChangeMode_angular_error;
		float traffic_dis_error;
		float othermode_dis_error;
		float othermode_angular_error;
		float target_index_V_ratio;
		float target_index_distance;
		std::string network_card;
    //int Car_packMode;



	//Hector SLAM pose
	Eigen::Vector3f slam_pose_, ukf_pose_;

	//Obs
	float obs_way_theta;
    bool isAvoidObs;
	bool OBS_limilMode;
	float all_obs_dis;
	float decay_obs_v;
	int AvoidObs_state;
	bool isFindObs;
	int last_subpath_index;
	bool isblind ;
	int avoid_way;
	bool obs_avoid_flag ;
	std::vector<Eigen::Vector3f> avoid_path;
	bool obs_return;
	bool isCloseNow;
	int command_OBSMode;


	//tracking
	int p_state_;
	int ready_path_index;
	std::vector<int> rpm;
    std::vector<int> theta;
	int now_path_index;
	int now_A_misson;
	int isALLfinish;

	int Command_STATE;
	int Command_id;

    static float v_buf ;


	std::vector<SUB_MISSONPATH> A_misson;//1個A_misson（任務鏈）含有幾個要停的任務

	std::vector<NODE_recv> Traffic_Node;

	trafficGO trafficGO_recv;



	std::vector<SUBPATH> AllSubPath;
    std::vector<NODE> NodeSet;
    float time_delay_counter;
    float time_delay_limit;

	bool ChangToTraffic;
	bool ChangToTraffic_finishstop;


	//error
	int LaserMiss;

	//protect
	bool protect_erase;
	bool v_stop ;
	bool traffic_send;


    //電梯
    bool ElevatorGO;
    int floor;
		bool changemap_finish;







};
float Move_Robot::v_buf = 0.04;
Move_Robot::Move_Robot(char *dev_name, int Baudrate)
{
    // LoadTitlePath();
    CarParameterPATH = TitlePath + CarParameterPATH_Local;
	ChangToTraffic = false;
	ChangToTraffic_finishstop = false;
    //local_parameter
    tracking_kp = 0.0;
    tracking_kd = 0.0;
		ChangeMode_dis_error = 0.0;
		ChangeMode_angular_error = 0.0;
		traffic_dis_error = 0.0;
		othermode_dis_error = 0.0;
		othermode_angular_error = 0.0;
		target_index_V_ratio = 0.0;
		target_index_distance = 0.0;
		network_card = " ";
    //Car_packMode = 0;

    std::string LocalparPATH = TitlePath + LocalparPATH_Local;
    sendreceive.Setmode(SetLocalpar(LocalparPATH));

     //Serial
     int baudrate = setBaudrate(Baudrate);
     if(SerialConnect(mySerial, dev_name, (speed_t)baudrate) < 0){

         ROS_INFO("Serial failed");
     }


    //Car_parameter
    CarName = "";
    Carnumber = -1;
    CarKind = -1;
    CarLength = 0.0;
    CarWidth = 0.0;
    LRWheeldis = 0.0;
    FBWheeldis = 0.0;
    LWheeldis = 0.0;
    Wheel_r = 0.0;
    Reduction_Ratio = 0.0;
    speedup = 0.0;
    speed = 0.0;
    JOYSTICK_SCALAR = 0.0;
    OBS_Front = 0.0;
    OBS_Side = 0.0;
    OBS_Front_limit = 0.0;
    OBS_Stop_Sec = 0.0;
    OBS_Wait_Sec = 0.0;
    OBS_Theta_Sec = 0.0;
    traffic_dis = 0.0;
    reMission_time = 0.0;

    LoadCarParameter(CarParameterPATH);


    for(int i=0;i<4;i++)
    {
        rpm.push_back(0);
        theta.push_back(0);
    }

    //joystick
    joystick_theta = 0.0;
    joystick_v = 0.0;

    btn_id = -1;

    //Laser
    p_base_frame_ ="base_link";

    //Hector SLAM pose
    slam_pose_ = Eigen::Vector3f(0,0,0);

    //UKF pose
    ukf_pose_ = Eigen::Vector3f(0,0,0);

    //Obs
    AvoidObs_state = P_OBS_NORMAL;
    obs_way_theta = 0.0;
    isAvoidObs = false;
    OBS_limilMode = false;
    all_obs_dis = 0.0;
    decay_obs_v = 0.0;
    isFindObs = false;
    last_subpath_index = 0;
    isblind = false;
    avoid_way = 0;
    obs_avoid_flag =false;
    isCloseNow = true;
    command_OBSMode = 0;
    obs_return =false;          //閉障回來


    //tracking
    time_delay_counter = 0;
    //time_delay_limit = 10.0/time_sample;
    p_state_ = P_STATE_IDLE;


    //PATH
    ready_path_index = 0;
    now_A_misson=0;
    now_path_index=0;


    //error
    LaserMiss = 0;



    //protect
    protect_erase = false;

    v_stop = false;

    //電梯
    ElevatorGO = false;
    floor = -1;
		changemap_finish = false;

    //ROS Subscriber & Publisher
    odometry_Publisher_ = node_.advertise<geometry_msgs::PoseStamped>("odom_speed", 1);
    Error_Publisher_= node_.advertise<std_msgs::Int8>("Error",10);
    v_Publisher_ = node_.advertise<std_msgs::Float32>("v", 5);
    //marker_Publisher_ = node_.advertise<visualization_msgs::Marker>("OBS_Marker", 1);
    STATE_Publisher_ = node_.advertise<move_robot::state>("STATE", 10);
    ReMissionState_Publisher_= node_.advertise<std_msgs::Int8>("ReMissionState", 10);
    marke_Publisher_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    Command_Publisher_ = node_.advertise<move_robot::setmap_ctr>("Command",10);
		Speak_Publisher_ = node_.advertise<std_msgs::String>("speak",10);

    //JoysickSubscriber_ = node_.subscribe("joystick", 5, &Move_Robot::joystickCallback, this);
    slamoutSubscriber_ = node_.subscribe("slam_out_pose", 5, &Move_Robot::slamoutCallback, this);
    ukfSubscriber_ = node_.subscribe("fusionUKF", 5, &Move_Robot::UKFCallback, this);
    //laserSubscriber_ = node_.subscribe("scan", 5, &Move_Robot::laserCallback, this);
    IdTypeSubscriber_=node_.subscribe("id", 5, &Move_Robot::IdTypeCallback, this);
    TrafficGOSubscriber_=node_.subscribe("GO", 5, &Move_Robot::TrafficGOCallback, this);
    //Clear_ObsModeSubscriber_=node_.subscribe("Clear_ObsMode", 5, &Move_Robot::ClearCallback, this);
    MissMsgSubscriber_ = node_.subscribe("Missing", 5, &Move_Robot::MissMsgCallback, this);
    TriggerSubscriber_=node_.subscribe("Trigger", 5, &Move_Robot::TriggerCallback_, this);
    floorSubscriber_=node_.subscribe("floor", 10, &Move_Robot::floorCallback_, this);




    //Time sampling time
    time_sample = 0.1;

    //Timer & Thread
    //control_timer = node_.createTimer(ros::Duration(time_sample), &Move_Robot::timerCallback, this);

}



Move_Robot::~Move_Robot()
{
	std::cout<<"Move_Robot delete"<<std::endl;
     if(receive_thread_)
         delete receive_thread_;
    if(receive_Battery_thread_)
         delete receive_Battery_thread_;
}

int Move_Robot::setBaudrate(int baudrate)
{
    if(baudrate == 50)
        return B50;
    else if(baudrate == 75)
        return B75;
    else if(baudrate == 110)
        return B110;
    else if(baudrate == 134)
        return B134;
    else if(baudrate == 150)
        return B150;
    else if(baudrate == 200)
        return B200;
    else if(baudrate == 300)
        return B300;
    else if(baudrate == 600)
        return B600;
    else if(baudrate == 1200)
        return B1200;
    else if(baudrate == 1800)
        return B1800;
    else if(baudrate == 2400)
        return B2400;
    else if(baudrate == 4800)
        return B4800;
    else if(baudrate == 9600)
        return B9600;
    else if(baudrate == 19200)
        return B19200;
    else if(baudrate == 38400)
        return B38400;
    else if(baudrate == 115200)
        return B115200;
    else
        return B0;
}

int Move_Robot::SerialConnect(SERIAL &serial, char *port_name, int speed)
{
    serial.serial_ok = false;

    serial.fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if(serial.fd < 0){

        std_msgs::Int8 msg;

        msg.data = 3;

        Error_Publisher_.publish(msg);

        std::cout<<"Error opening"<<std::endl;
        return -1;
    }

    if(tcgetattr(serial.fd, &(serial.oldtio)) < 0){
        std::cout<<"Error from tcgetattr"<<std::endl;

        std_msgs::Int8 msg;

        msg.data = 4;

        Error_Publisher_.publish(msg);
        return -1;
    }

    cfsetospeed(&serial.oldtio, (speed_t)speed);
    cfsetispeed(&serial.oldtio, (speed_t)speed);

    serial.oldtio.c_cflag |= (CLOCAL | CREAD);
    serial.oldtio.c_cflag &= ~CSIZE;
    serial.oldtio.c_cflag |= CS8;
    serial.oldtio.c_cflag &= ~PARENB;
    serial.oldtio.c_cflag &= ~CSTOPB;
    serial.oldtio.c_cflag &= ~CRTSCTS;


    serial.oldtio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    serial.oldtio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    serial.oldtio.c_oflag &= ~OPOST;

    serial.oldtio.c_cc[VMIN] = 1;
    serial.oldtio.c_cc[VTIME] = 1;

    if(tcsetattr(serial.fd, TCSANOW, &serial.oldtio) != 0){
        std::cout<<"Error from tcgetattr"<<std::endl;
        return -1;
        std_msgs::Int8 msg;

        msg.data = 4;

        Error_Publisher_.publish(msg);
    }

    serial.serial_ok = true;

    std::cout<<"Serial Opened"<<std::endl;

    return 0;
}

void Move_Robot::LoadCarParameter(std::string file_buf)
{

    std::fstream fin;

    char *file = const_cast<char *>(file_buf.c_str());
    fin.open(file, std::fstream::in);
    if(!fin.is_open())
    {
        ROS_INFO("Error: CarParameter is not opened!!   123");
        std::cout << "> " << file_buf.c_str() << std::endl;
    }
    else{
        ROS_INFO("the file is opened!!");

        std::string s_CarName;
        std::getline(fin, s_CarName);
        std::getline(fin, s_CarName);
        CarName = s_CarName;

        std::string s_Carnumber;
        std::getline(fin, s_Carnumber);
        std::getline(fin, s_Carnumber);
        Carnumber = std::atoi(s_Carnumber.c_str());

        std::string s_CarKind;
        std::getline(fin, s_CarKind);
        std::getline(fin, s_CarKind);
        CarKind = std::atoi(s_CarKind.c_str());

        std::string s_CarLength;
        std::getline(fin, s_CarLength);
        std::getline(fin, s_CarLength);
        CarLength = std::atof(s_CarLength.c_str());

        std::string s_CarWidth;
        std::getline(fin, s_CarWidth);
        std::getline(fin, s_CarWidth);
        CarWidth = std::atof(s_CarWidth.c_str());

        std::string s_LRWheeldis;
        std::getline(fin, s_LRWheeldis);
        std::getline(fin, s_LRWheeldis);
        LRWheeldis = std::atof(s_LRWheeldis.c_str());

        std::string s_FBWheeldis;
        std::getline(fin, s_FBWheeldis);
        std::getline(fin, s_FBWheeldis);
        FBWheeldis = std::atof(s_FBWheeldis.c_str());

        std::string s_LWheeldis;
        std::getline(fin, s_LWheeldis);
        std::getline(fin, s_LWheeldis);
        LWheeldis = std::atof(s_LWheeldis.c_str());

        std::string s_Wheel_r;
        std::getline(fin, s_Wheel_r);
        std::getline(fin, s_Wheel_r);
        Wheel_r = std::atof(s_Wheel_r.c_str());

        std::string s_Reduction_Ratio;
        std::getline(fin, s_Reduction_Ratio);
        std::getline(fin, s_Reduction_Ratio);
        Reduction_Ratio = std::atof(s_Reduction_Ratio.c_str());

        std::string s_speedup;
        std::getline(fin, s_speedup);
        std::getline(fin, s_speedup);
        speedup = std::atof(s_speedup.c_str());

        std::string s_speed;
        std::getline(fin, s_speed);
        std::getline(fin, s_speed);
        speed = std::atof(s_speed.c_str());

        //JOYSTICK

        std::string s_JOYSTICK_SCALAR;
        std::getline(fin, s_JOYSTICK_SCALAR);
        std::getline(fin, s_JOYSTICK_SCALAR);
        JOYSTICK_SCALAR = std::atof(s_JOYSTICK_SCALAR.c_str());


        //OBS

        std::string s_OBS_Front;
        std::getline(fin, s_OBS_Front);
        std::getline(fin, s_OBS_Front);
        OBS_Front = std::atof(s_OBS_Front.c_str());

        std::string s_OBS_Side;
        std::getline(fin, s_OBS_Side);
        std::getline(fin, s_OBS_Side);
        OBS_Side = std::atof(s_OBS_Side.c_str());

        std::string s_OBS_Front_limit;
        std::getline(fin, s_OBS_Front_limit);
        std::getline(fin, s_OBS_Front_limit);
        OBS_Front_limit = std::atof(s_OBS_Front_limit.c_str());

        std::string s_OBS_Stop_Sec;
        std::getline(fin, s_OBS_Stop_Sec);
        std::getline(fin, s_OBS_Stop_Sec);
        OBS_Stop_Sec = std::atof(s_OBS_Stop_Sec.c_str());

        std::string s_OBS_Wait_Sec;
        std::getline(fin, s_OBS_Wait_Sec);
        std::getline(fin, s_OBS_Wait_Sec);
        OBS_Wait_Sec = std::atof(s_OBS_Wait_Sec.c_str());

        std::string s_OBS_Theta_Sec;
        std::getline(fin, s_OBS_Theta_Sec);
        std::getline(fin, s_OBS_Theta_Sec);
        OBS_Theta_Sec = std::atof(s_OBS_Theta_Sec.c_str());

        //traffic
        std::string s_traffic_dis;
        std::getline(fin, s_traffic_dis);
        std::getline(fin, s_traffic_dis);
        traffic_dis = std::atof(s_traffic_dis.c_str());

        std::string s_reMission_time;
        std::getline(fin, s_reMission_time);
        std::getline(fin, s_reMission_time);
        reMission_time = std::atof(s_reMission_time.c_str());



        CarLength_helf = CarLength/2.0;
        CarWidth_helf = CarWidth/2.0;
        LRWheeldis_helf = LRWheeldis/2.0;
        FBWheeldis_helf = FBWheeldis/2.0;

        std::cout<<" CarLength_helf  "<<CarLength_helf<<std::endl;
        std::cout<<" CarWidth_helf  "<<CarWidth_helf<<std::endl;
        std::cout<<" LRWheeldis_helf  "<<LRWheeldis_helf<<std::endl;
        std::cout<<" FBWheeldis_helf  "<<FBWheeldis_helf<<std::endl;


        Car.setCar_Par( LWheeldis,  Wheel_r,  Reduction_Ratio,  FBWheeldis_helf,  LRWheeldis_helf);






//        std::cout<<"s_CarWidth  "<<CarWidth<<std::endl;
//        std::cout<<"LRWheeldis  "<<LRWheeldis<<std::endl;
//        std::cout<<"FBWheeldis  "<<FBWheeldis<<std::endl;
//        std::cout<<"LWheeldis  "<<LWheeldis<<std::endl;
//        std::cout<<"Wheel_r  "<<Wheel_r<<std::endl;
//        std::cout<<"Reduction_Ratio  "<<Reduction_Ratio<<std::endl;
//        std::cout<<"speedup  "<<speedup<<std::endl;

    }

    fin.close();


}
int Move_Robot::checkDeviceNum(const sensor_msgs::LaserScan& scan)
{

    int range_size = scan.ranges.size();

    float angle_min = (scan.angle_min)*180.0/ M_PI;

    float angle_max = (scan.angle_max)*180.0/ M_PI;

    float angle_increment = scan.angle_increment*180.0/ M_PI;

    float angle_range = fabs(angle_max - angle_min);

    float checksize = angle_range/angle_increment;



    if((range_size/checksize) > 1.5){
        return 2;
    }
    else{
        return 1;
    }

}
// void Move_Robot::ClearCallback(const std_msgs::Int8& msg)
// {
//     int obs_clear_buf = msg.data;
//     std::vector<SUB_MISSONPATH_SUBPOINT> zzz;

//     switch (obs_clear_buf) {
//     case Clear_Mode:
//         int xxx;
//         p_state_ = P_STATE_IDLE;
//         A_misson.clear();
//         A_misson[ready_path_index].ALL_pathnode.clear();
//         memset(&trafficGO_recv,0,sizeof(trafficGO_recv));

//         now_A_misson=0;
//         now_path_index=0;
//         ready_path_index=0;
//         Tracking_Angle_Init(xxx, true);
//         Tracking_Trajectory(xxx, true);
//         Trajectory_Tracking(xxx, true);
//         Navigation_move(zzz,true);

//         AvoidObs_state = P_OBS_NORMAL;
//         obs_way_theta = 0.0;
//         isAvoidObs = false;
//         OBS_limilMode = false;
//         all_obs_dis = 0.0;
//         decay_obs_v = 0.0;
//         isFindObs = false;
//         last_subpath_index = 0;
//         isblind = false;
//         avoid_way = 0;
//         obs_avoid_flag =false;
//         isCloseNow = true;
//         command_OBSMode = 0;
//         obs_return =false;          //閉障回來
//         avoid_path.clear();

//         protect_erase = true;
//         traffic_send = false;


//         break;
//     case Close_AllObs:
//         command_OBSMode = 1;

//         break;
//     case limitObs_Mode:
//         command_OBSMode = 2;

//         break;
//     case Normal_Mode:
//         command_OBSMode = 0;

//         break;
//     default:
//         break;
//     }


//     std::cout<<"A_misson.size  clear"<<A_misson.size() <<std::endl;
// }
// void Move_Robot::TriggerCallback(const std_msgs::Int8& msg)
// {
//     int Trigger = msg.data;

//     switch(Trigger){
//             case ReloadCarParameter:
//                  LoadCarParameter(CarParameterPATH);
//             break;

//             default:
//             break;


//     }


// }
void Move_Robot::floorCallback_(const std_msgs::Int8& msg)
{
		std::cout<<"floorCallback_"<<std::endl;
		int floor_loc = msg.data;
		switch(floor_loc)
		{
				case 1:
					std::cout<<"floor one arrived"<<std::endl;
                    floor = floor_loc;
				break;
				case 8:
					std::cout<<"floor eight arrived"<<std::endl;
                    floor = floor_loc;
				break;

				case 100:
					std::cout<<"finish change"<<std::endl;
                    changemap_finish = true;
				break;

				default:
				break;
		}
}
void Move_Robot::TriggerCallback_(const std_msgs::Int8& msg)
{
		std::cout<<"move_robot.h delete"<<std::endl;
		int Trigger = msg.data;
		switch(Trigger)
		{
				case ReloadCarParameter:
							delete this;
				break;

				default:
				break;
		}
}
void Move_Robot::TrafficGOCallback(const move_robot::traffic_recv& command)
{

    trafficGO_recv.id = command.id;
    trafficGO_recv.GO = command.GO;
    trafficGO_recv.speed = command.speed;

    if(trafficGO_recv.GO == 3)
    {
        trafficGO_recv.speed = command.speed;
    }

    std::cout<<"command.id "<<command.id<<std::endl;
    std::cout<<"command.GO "<<command.GO<<std::endl;
    std::cout<<"command.speed "<<command.speed<<std::endl;

    std::cout<<"trafficGO_recv.id "<<trafficGO_recv.id<<std::endl;
    std::cout<<"trafficGO_recv.GO "<<trafficGO_recv.GO<<std::endl;
    std::cout<<"trafficGO_recv.speed "<<trafficGO_recv.speed<<std::endl;



}
void Move_Robot::IdTypeCallback(const move_robot::Node_recv& command)
{



    static int error_cnt = 0;

     if(protect_erase)
    {
        protect_erase = false;
    }

    int value = command.value;
    int check = command.check;

    NODE_recv node_buf;

    node_buf.id=command.id;
    node_buf.type=command.type;
    node_buf.kin=command.kin;
    node_buf.time=command.time;
    node_buf.btn_finish=command.btn_finish;
    node_buf.node_pose.x()=command.x;
    node_buf.node_pose.y()=command.y;
    node_buf.node_pose.z()=command.z;
    node_buf.line=command.line;
    node_buf.radius=command.radius;
    node_buf.map=command.map;
    node_buf.floor=command.floor;

    //  std::cout<<"node_buf.id "<<node_buf.id<<std::endl;
    //             std::cout<<"node_buf.node_pose.x() "<<node_buf.node_pose.x()<<std::endl;
    //              std::cout<<"node_buf.node_pose.y() "<<node_buf.node_pose.y()<<std::endl;
    //               std::cout<<"node_buf.node_pose.z() "<<node_buf.node_pose.z()<<std::endl;

   Traffic_Node.push_back(node_buf);

   //保護漏封包問題
   if(Traffic_Node.size()!=check)  {
       Traffic_Node.clear();
       std_msgs::Int8 msg;

       msg.data = 5;
       error_cnt ++;

       if(error_cnt == 10)
       {
            Error_Publisher_.publish(msg);
            error_cnt = 0;
       }

   }

    //保護
    if(Traffic_Node.size()==value)
    {
        bool protect_send = false;

        if(A_misson.size() > 0)
        {
            //保護送到原地
            if(A_misson[A_misson.size()-1].ALL_pathnode[0].id == Traffic_Node[0].id)
            {
                 Traffic_Node.clear();
                 protect_send = true ;
            }
        }

        SUB_MISSONPATH sub_missonPath_buf;


        if(!protect_send)
        {
            error_cnt = 0;
            Misson_Path(sub_missonPath_buf);
            if(!mission_dont_push)
            {
                A_misson.push_back(sub_missonPath_buf);


            }
            else
            {
                std_msgs::Int8 msg;
			    msg.data = 10;
			    Error_Publisher_.publish(msg);
                mission_dont_push = false;
            }

            Traffic_Node.clear();

            if(A_misson.size() == 1)
                p_state_ = P_STATE_MOVE;

        }

    }


}
void Move_Robot::Misson_Path(SUB_MISSONPATH &sub_missonPath_buf)
{
    std::cout<<" Misson_Path "<<std::endl;
    std::vector<Eigen::Vector3f> id_Point_inPath;   //路徑上每個點的座標
    std::vector<int> id_Point_lineMode;   //路徑上每個線的模式
    std::vector<float> id_Point_radius;   //路徑上每個R


    id_Point_inPath.push_back(Traffic_Node[0].node_pose);
    id_Point_lineMode.push_back(Traffic_Node[0].line);
    id_Point_radius.push_back(Traffic_Node[0].radius);


    for(int i=0;i<Traffic_Node.size();i++)
    {
    	sub_missonPath_buf.ALL_pathnode.push_back(Traffic_Node[i]);

    }

    int start = Traffic_Node[0].id;
	int start_type = Traffic_Node[0].type;

    std::string MAP = Traffic_Node[0].map;

    for(int i=1;i<Traffic_Node.size();i++)
    {

        if(MAP == Traffic_Node[i].map)
        {
            if(Traffic_Node[i].type!=1 ) //type 1 =P
            {
                SUB_MISSONPATH_SUBPOINT sub_missonPath_subPoint_buf;

                int end=Traffic_Node[i].id;
                sub_missonPath_subPoint_buf.start=start;
                sub_missonPath_subPoint_buf.start_type=start_type;
                sub_missonPath_subPoint_buf.end=end;
                sub_missonPath_subPoint_buf.end_type=Traffic_Node[i].type;
                sub_missonPath_subPoint_buf.map = MAP;


								std::cout<<"Traffic_Node[i].type "<<Traffic_Node[i].type<<std::endl;
                if(Traffic_Node[i].type == MISSON_Elevator_Inside)
                {
                    sub_missonPath_subPoint_buf.floor = Traffic_Node[i].floor;
										//std::cout<<"floor "<<sub_missonPath_subPoint_buf.floor<<std::endl;
                }
								std::cout<<"floor "<<sub_missonPath_subPoint_buf.floor<<std::endl;

                id_Point_inPath.push_back(Traffic_Node[i].node_pose);
                id_Point_lineMode.push_back(Traffic_Node[i].line);
                id_Point_radius.push_back(Traffic_Node[i].radius);


                CreatPath(id_Point_inPath,id_Point_lineMode,id_Point_radius,sub_missonPath_subPoint_buf);

                sub_missonPath_buf.sub_missonPath.push_back(sub_missonPath_subPoint_buf);

                std::cout<<"start "<<sub_missonPath_subPoint_buf.start<<std::endl;
                std::cout<<"end "<<sub_missonPath_subPoint_buf.end<<std::endl;



                start=end;
                start_type=Traffic_Node[i].type;
                id_Point_inPath.clear();
                id_Point_inPath.push_back(Traffic_Node[i].node_pose);

                id_Point_lineMode.clear();
                id_Point_lineMode.push_back(Traffic_Node[i].line);

                id_Point_radius.clear();
                id_Point_radius.push_back(Traffic_Node[i].radius);

            }
            else //如果=P //(沒有了)遇到passing點 再前後新增NODE點來產生貝茲 避免交管點到下個轉彎彎不過去
            {

                id_Point_inPath.push_back(Traffic_Node[i].node_pose);
                id_Point_lineMode.push_back(Traffic_Node[i].line);
                id_Point_radius.push_back(Traffic_Node[i].radius);

            }

        }
        else
        {

            if(id_Point_inPath.size() > 1)
            {
                std::cout<<"error line mode"<<std::endl;
                mission_dont_push = true;
                break;
            }

            MAP = Traffic_Node[i].map;
            id_Point_inPath.clear();
            id_Point_inPath.push_back(Traffic_Node[i].node_pose);

            id_Point_lineMode.clear();
            id_Point_lineMode.push_back(Traffic_Node[i].line);

            id_Point_radius.clear();
            id_Point_radius.push_back(Traffic_Node[i].radius);

            start = Traffic_Node[i].id;
	        start_type = Traffic_Node[i].type;

        }


    }
    id_Point_inPath.clear();
}
void Move_Robot::CreatPath(std::vector<Eigen::Vector3f> input_buf,std::vector<int> &id_Point_lineMode,std::vector<float> radius, SUB_MISSONPATH_SUBPOINT &return_buf)
{
    std::cout<<" CreatPath "<<std::endl;
    //bool error_break = false;
    int count_index = 0;
    for(int i=0; i<input_buf.size()-1; i=count_index){

        //std::cout<<"input_buf.size() "<<input_buf.size()<<std::endl;
        // std::cout<<"input_buf "<<input_buf[0]<<std::endl;
        // std::cout<<"input_buf "<<input_buf[input_buf.size()-1]<<std::endl;
        // std::cout<<"return_buf.start()= "<<return_buf.start<<std::endl;
        // std::cout<<"return_buf.end()= "<<return_buf.end<<std::endl;

        // std::cout<<"count_index "<<count_index<<std::endl;

        // std::cout<<"id_Point_lineMode.size() "<<id_Point_lineMode.size()<<std::endl;

        // std::cout<<"id_Point_lineMode[count_index] "<<id_Point_lineMode[0]<<std::endl;
        // std::cout<<"id_Point_lineMode[count_index] "<<id_Point_lineMode[1]<<std::endl;
        // std::cout<<"id_Point_lineMode[count_index] "<<id_Point_lineMode[2]<<std::endl;

        std::vector<Eigen::Vector3f> sub_path;
        Eigen::Vector3f p1;
        Eigen::Vector3f p1_;
        Eigen::Vector3f p2;
        Eigen::Vector3f p2_;
        Eigen::Vector3f p3;
        // std::vector<Eigen::Vector3f> curve_input;



        // int check_number = 0;
        //float sample_dis = 0.5;//50cm
        float sample_dis = 0.04;//4cm
        float t_step = 0.0;
        float dis = 0.0;
        float step = 0.0;
        float dis_limit = 0;


        //給畫弧
        float ang1 = 0.0;
        float ang2 = 0.0;
        float diff_ang = 0.0;
        //float Arcradius = 0.5;

        float Arcradius = radius[count_index];
        float shift_len = 0.0;
        float dx = 0.0;
        float dy = 0.0;
        float unit_x = 0.0;
        float unit_y = 0.0;
        float dis_1 = 0.0;
        float dis_1_ = 0.0;
        float dis_2 =  0.0;
        float t_step1 = 0.0;
        float t_step1_ = 0.0;
        float t_step2 = 0.0;
        float step1 = 0.0;
        float step1_ = 0.0;
        float step2 = 0.0;


        Eigen::Vector2f Cornervector;
        switch(id_Point_lineMode[count_index]){
        case 0://直線
            for(int j=0;j<2;j++)
            {
                if(i+j == input_buf.size()-1)
                {
                    // if(j<1)
                    // {
                    //     return_buf.sub_missonPath_subPoint.clear();
                    //     mission_dont_push = true;
                    //     break;
                    // }
                }
                Eigen::Vector3f sub_p = input_buf[i+j];
                sub_path.push_back(sub_p);
            }

            p1 = sub_path[0];
            p2 = sub_path[1];
            // //check_number = 1;
            dis = getDistance(p1.x(), p1.y(), p2.x(), p2.y());
            t_step = dis/sample_dis;
            step = 1/t_step;

            //if(dis > dis_limit){
            for(float t=0; t<1; t+=step){
                Eigen::Vector3f newP(0,0,0);
                newP[0] = (1-t)*p1[0] + t*p2[0];
                newP[1] = (1-t)*p1[1] + t*p2[1];
                newP[2] = (1-t)*p1[2] + t*p2[2];
                return_buf.sub_missonPath_subPoint.push_back(newP);
            }




             count_index +=1;
             //std::cout<<"count_index "<<count_index<<std::endl;

        break;
        case 1://Arc
            if(input_buf.size() - count_index < 3)
            {
                // std_msgs::Int8 msg;
                // msg.data = 11;
                // Error_Publisher_.publish(msg);
                std::cout<<"arc error"<<std::endl;
                std::cout<<"return_buf.start()="<<return_buf.start<<std::endl;
                std::cout<<"return_buf.end()="<<return_buf.end<<std::endl;
                return_buf.sub_missonPath_subPoint.clear();
                mission_dont_push = true;
                //error_break = true;
                break;
            }
            for(int j=0;j<3;j++)
            {
                // if(i+j == input_buf.size()-1)
                // {
                //     if(j<2)
                //     {
                //         return_buf.sub_missonPath_subPoint.clear();
                //         mission_dont_push = true;
                //         break;
                //     }
                // }
                Eigen::Vector3f sub_p = input_buf[i+j];
                sub_path.push_back(sub_p);
            }

            p1 = sub_path[0];
            p2 = sub_path[1];
            p3 = sub_path[2];


            ang1 = atan2(-(p1.y() - p2.y()),p1.x() - p2.x());
            ang2 = atan2(-(p3.y() - p2.y()),p3.x() - p2.x());
            diff_ang = ((int)(180*ang2/M_PI - 180*ang1/M_PI) + 360) % 360;
            if(diff_ang > 180) diff_ang = 360 - diff_ang;
            shift_len = Arcradius / tan(M_PI * (diff_ang * 0.5)/180);

            //畫第一個虛擬點
            dx = p2.x()-p1.x();
            dy = p2.y()-p1.y();

            unit_x = dx /getDistance(p1.x(),p1.y(),p2.x(),p2.y());
            unit_y = dy /getDistance(p1.x(),p1.y(),p2.x(),p2.y());

            Cornervector << shift_len *unit_x , shift_len *unit_y;

            p1_ << p2.x()-Cornervector.x() ,p2.y()-Cornervector.y() ,p1.z();






            //畫第二個虛擬點
            dx = p3.x()-p2.x();
            dy = p3.y()-p2.y();

            unit_x = dx /getDistance(p3.x(),p3.y(),p2.x(),p2.y());
            unit_y = dy /getDistance(p3.x(),p3.y(),p2.x(),p2.y());

            Cornervector << shift_len *unit_x , shift_len *unit_y;

            p2_ << p2.x()+Cornervector.x() ,p2.y()+Cornervector.y() ,p2.z();


            dis_1 = getDistance(p1_[0], p1_[1], p1[0], p1[1]);
            dis_2 = getDistance(p3[0], p3[1], p2_[0], p2_[1]);
            dis_1_= Arcradius * diff_ang *M_PI /180.0;
            t_step1 = dis_1/sample_dis;
            t_step2 = dis_2/sample_dis;
            t_step1_= dis_1_/sample_dis;
            step1 = 1/t_step1;
            step2 = 1/t_step2;
            step1_ = 1/t_step1_;


            //畫線
            for(float t=0; t<1; t+=step1){
                Eigen::Vector3f newP(0,0,0);
                newP[0] = (1-t)*p1[0] + t*p1_[0];
                newP[1] = (1-t)*p1[1] + t*p1_[1];
                newP[2] = (1-t)*p1[2] + t*p1_[2];
                return_buf.sub_missonPath_subPoint.push_back(newP);
            }

            //畫ARC
            for(float t=0; t<1; t+=step1_){
                Eigen::Vector3f newP(0,0,0);
                newP[0] = (1-t)*(1-t)*p1_[0] + 2*t*(1-t)*p2[0] + t*t*p2_[0];
                newP[1] = (1-t)*(1-t)*p1_[1] + 2*t*(1-t)*p2[1] + t*t*p2_[1];
                newP[2] = (1-t)*(1-t)*p1_[2] + 2*t*(1-t)*p2[2] + t*t*p2_[2];
                return_buf.sub_missonPath_subPoint.push_back(newP);
            }

            //畫線
            for(float t=0; t<1; t+=step2){
                Eigen::Vector3f newP(0,0,0);
                newP[0] = (1-t)*p2_[0] + t*p3[0];
                newP[1] = (1-t)*p2_[1] + t*p3[1];
                newP[2] = (1-t)*p2_[2] + t*p3[2];
                return_buf.sub_missonPath_subPoint.push_back(newP);
            }



            count_index +=2;

        break;
        case 2://circle
            for(int j=0;j<3;j++)
            {
                if(i+j == input_buf.size()-1)
                {
                    if(j<2)
                    {
                        return_buf.sub_missonPath_subPoint.clear();
                        mission_dont_push = true;
                        break;
                    }
                }
                Eigen::Vector3f sub_p = input_buf[i+j];
                sub_path.push_back(sub_p);
            }

            p1 = sub_path[0];
            p2 = sub_path[1];
            p3 = sub_path[2];

            if(fabs(p2.y()-p1.y())<fabs(p2.y()-p3.y()))
            {
                p2.y()=p1.y();
                p2.x()=p3.x();

            }
            else
            {
                p2.y()=p3.y();
                p2.x()=p1.x();
            }


            ang1 = atan2(-(p1.y() - p2.y()),p1.x() - p2.x());
            ang2 = atan2(-(p3.y() - p2.y()),p3.x() - p2.x());
            diff_ang = (int)(180*ang2/M_PI - 180*ang1/M_PI +360) % 360;
            if(diff_ang > 180) diff_ang = 360 - diff_ang;
            shift_len = Arcradius / tan(M_PI * (diff_ang * 0.5)/180);

            //畫第一個虛擬點
            dx = p2.x()-p1.x();
            dy = p2.y()-p1.y();

            unit_x = dx /getDistance(p1.x(),p1.y(),p2.x(),p2.y());
            unit_y = dy /getDistance(p1.x(),p1.y(),p2.x(),p2.y());

            Cornervector << shift_len *unit_x , shift_len *unit_y;

            p1_ << p2.x()-Cornervector.x() ,p2.y()-Cornervector.y() ,p1.z();





            //畫第二個虛擬點
            dx = p3.x()-p2.x();
            dy = p3.y()-p2.y();

            unit_x = dx /getDistance(p3.x(),p3.y(),p2.x(),p2.y());
            unit_y = dy /getDistance(p3.x(),p3.y(),p2.x(),p2.y());

            Cornervector << shift_len *unit_x , shift_len *unit_y;

            p2_ << p2.x()+Cornervector.x() ,p2.y()+Cornervector.y() ,p2.z();


            dis_1 = getDistance(p1_[0], p1_[1], p1[0], p1[1]);
            dis_2 = getDistance(p2_[0], p2_[1], p3[0], p3[1]);
            dis_1_= Arcradius * diff_ang *M_PI /180.0;

            t_step1 = dis_1/sample_dis;
            t_step2 = dis_2/sample_dis;
            t_step1_= dis_1_/sample_dis;
            step1 = 1/t_step1;
            step2 = 1/t_step2;
            step1_ = 1/t_step1_;


            //畫線
            for(float t=0; t<1; t+=step1){
                Eigen::Vector3f newP(0,0,0);
                newP[0] = (1-t)*p1[0] + t*p1_[0];
                newP[1] = (1-t)*p1[1] + t*p1_[1];
                newP[2] = (1-t)*p1[2] + t*p1_[2];
                return_buf.sub_missonPath_subPoint.push_back(newP);
            }
            //畫ARC
            for(float t=0; t<1; t+=step1_){
                Eigen::Vector3f newP(0,0,0);
                newP[0] = (1-t)*(1-t)*p1_[0] + 2*t*(1-t)*p2[0] + t*t*p2_[0];
                newP[1] = (1-t)*(1-t)*p1_[1] + 2*t*(1-t)*p2[1] + t*t*p2_[1];
                newP[2] = (1-t)*(1-t)*p1_[2] + 2*t*(1-t)*p2[2] + t*t*p2_[2];
                return_buf.sub_missonPath_subPoint.push_back(newP);
            }



            //畫線
            for(float t=0; t<1; t+=step2){
                Eigen::Vector3f newP(0,0,0);
                newP[0] = (1-t)*p2_[0] + t*p3[0];
                newP[1] = (1-t)*p2_[1] + t*p3[1];
                newP[2] = (1-t)*p2_[2] + t*p3[2];
                return_buf.sub_missonPath_subPoint.push_back(newP);
            }

            // std::cout<<"dis_1 "<< dis_1<<std::endl;
            // std::cout<<"dis_2 "<< dis_2<<std::endl;
            // std::cout<<"dis_1_ "<< dis_1_<<std::endl;

            // std::cout<<"p1 "<< p1<<std::endl;
            // std::cout<<"p2 "<< p2<<std::endl;
            // std::cout<<"p3 "<< p3<<std::endl;
            // std::cout<<"p1_ "<< p1_<<std::endl;
             //std::cout<<"return_buf.sub_missonPath_subPoint "<< return_buf.sub_missonPath_subPoint[return_buf.sub_missonPath_subPoint.size() - 1]<<std::endl;




            count_index +=2;
        break;
        }
        //std::cout<<"input_buf "<<input_buf[input_buf.size()-1]<<std::endl;

         //std::cout<<"return_buf.sub_missonPath_subPoint "<< return_buf.sub_missonPath_subPoint[return_buf.sub_missonPath_subPoint.size() - 1]<<std::endl;
        if(mission_dont_push == true)
        {
            break;
        }

     }
		 return_buf.sub_missonPath_subPoint.push_back(input_buf[input_buf.size()-1]);



}
float Move_Robot::getDistance(float x1, float y1, float x2, float y2)
{

    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
float Move_Robot::FindAngle(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3)
{

    float v1_x = p2.x() - p1.x();
    float v1_y = p2.y() - p1.y();

    float v2_x = p3.x() - p2.x();
    float v2_y = p3.y() - p2.y();

    float cos_t = (v1_x*v2_x + v1_y*v2_y)/(sqrt(v1_x*v1_x + v1_y*v1_y)*sqrt(v2_x*v2_x + v2_y*v2_y));
    float theta = acos(cos_t);
    return theta;
}

void Move_Robot::Misson_state(bool isReSet)
{

	static bool send_changemap = false;
	static bool send_speak = false;
	static int cnt_traffic = 0;
	static int cnt_virtual_traffic = 0;

	int cnt_traffic_limit;
	int cnt_virtual_traffic_limit;
	if(!isReSet)
	{

    ROS_INFO("Misson_state ");



    int type = A_misson[now_A_misson].sub_missonPath[now_path_index].end_type;
    int id = A_misson[now_A_misson].sub_missonPath[now_path_index].end;

    std::string Change_Map_name;
    Eigen::Vector3f ini_pose;

    std::cout<<"type  " <<type<<std::endl;
    Command_id = id;

		if(ChangToTraffic_finishstop)
		{
				type = MISSON_traffic;
				std::cout<<"ChangToTraffic_finishstop"<<std::endl;
		}


    switch(type){
        case MISSON_traffic:
            std::cout<<"MISSON_traffic" <<std::endl;
            Command_STATE = Command_STATE_Traffic;

            //停在交管點上 使它前往下一個任務
            if(trafficGO_recv.id==id)
                if(trafficGO_recv.GO ==1)
                {
					ChangToTraffic_finishstop = false;
                    p_state_ = P_STATE_MOVE;
                    trafficGO_recv.GO = 0 ;
                   memset(&trafficGO_recv,0,sizeof(trafficGO_recv));
                   std::cout<<"===State_Machine MISSON_traffic==="<<std::endl;
                }

            //重派 時間到發送


            cnt_traffic_limit = reMission_time/time_sample ;
            if(cnt_traffic < cnt_traffic_limit)
                cnt_traffic++;
            else
            {
				if(reMission_time > reMission_time_freq)
				{
                	cnt_traffic = (reMission_time/time_sample) - (reMission_time_freq/time_sample);
				}
				else
				{
					cnt_traffic = 0;
				}
                std_msgs::Int8 msg;

                msg.data = 1;

                ReMissionState_Publisher_.publish(msg);
            }


            //如果交管為最後 依然可以轉換為IDLE （配合交管使用）
            if(ready_path_index >= A_misson.size())
            {
                ready_path_index =0;
                A_misson[ready_path_index].ALL_pathnode.clear();
                A_misson.clear();
                p_state_=P_STATE_IDLE;
                isALLfinish =false;
								ChangToTraffic_finishstop = false;
                ROS_INFO("All Success!!!\n");


            }

        break;

        case MISSON_Loading_Time:

            Command_STATE = Command_STATE_Loading;
            std::cout<<"MISSON_Loading_Time" <<std::endl;

            for(int i=0;i<A_misson[now_A_misson].ALL_pathnode.size();i++){
                if(A_misson[now_A_misson].ALL_pathnode[i].id==id)
                {
                    float time = A_misson[now_A_misson].ALL_pathnode[i].time ;
                    time_delay_limit = time/time_sample;
                    break;
                }
            }
            p_state_ = P_STATE_TIME_DELAY;
            //p_state_ = P_STATE_MOVE;
        break;

        case MISSON_Loading_Interrupt:
            Command_STATE = Command_STATE_Loading;
            std::cout<<"MISSON_Loading_Interrupt" <<std::endl;
            p_state_ = P_STATE_MOVE;
            std::cout<<"===State_Machine MISSON_Loading_Interrupt==="<<std::endl;
        break;

        case MISSON_unLoading_Time:
            Command_STATE = Command_STATE_unLoading;
            std::cout<<"MISSON_unLoading_Time" <<std::endl;
            for(int i=0;i<A_misson[now_A_misson].ALL_pathnode.size();i++)
                if(A_misson[now_A_misson].ALL_pathnode[i].id==id)
                {
                    float time = A_misson[now_A_misson].ALL_pathnode[i].time;
                    std::cout<<"ALL_pathnode[i].time  " << A_misson[now_A_misson].ALL_pathnode[i].time<<std::endl;
                    time_delay_limit = time/time_sample;
                    break;
                }
            p_state_ = P_STATE_TIME_DELAY;
            //p_state_ = P_STATE_MOVE;
        break;

        case MISSON_unLoading_Interrupt:
            Command_STATE = Command_STATE_unLoading;
            std::cout<<"MISSON_unLoading_Interrupt" <<std::endl;
            p_state_ = P_STATE_MOVE;
            std::cout<<"===State_Machine MISSON_unLoading_Interrupt==="<<std::endl;
        break;

        case MISSON_ChangeMode:
            Command_STATE = Command_STATE_ChangeMode;
            std::cout<<"MISSON_ChangeMode" <<std::endl;

		        ROS_INFO(" Success!!!\n");
                p_state_ = P_STATE_MOVE;

                if(ready_path_index >= A_misson.size())
                {
                    ready_path_index =0;
                    A_misson[ready_path_index].ALL_pathnode.clear();
                    A_misson.clear();
                    p_state_=P_STATE_IDLE;
                    isALLfinish =false;
										ChangToTraffic_finishstop = false;
                    ROS_INFO("All Success!!!\n");

                }

        break;

        case MISSON_TemporaryMode:
            Command_STATE = Command_STATE_TemporaryMode;
            std::cout<<"MISSON_TemporaryMode" <<std::endl;

		    ROS_INFO(" Success!!!\n");
                p_state_ = P_STATE_MOVE;

                if(ready_path_index >= A_misson.size())
                {
                    ready_path_index =0;
                    A_misson[ready_path_index].ALL_pathnode.clear();
                    A_misson.clear();
                    p_state_= P_STATE_MISSON;
                    isALLfinish =false;
										ChangToTraffic_finishstop = false;
                    ROS_INFO("All Success!!!\n");
                    cnt_traffic = 0;
                    std_msgs::Int8 msg;
                    msg.data = 2;
                    ReMissionState_Publisher_.publish(msg);

                }

        break;

        case MISSON_FreeLoading:

            Command_STATE = Command_STATE_FreeLoading;
            std::cout<<"MISSON_FreeLoading" <<std::endl;

            for(int i=0;i<A_misson[now_A_misson].ALL_pathnode.size();i++){
                if(A_misson[now_A_misson].ALL_pathnode[i].id==id)
                {
                    float time = A_misson[now_A_misson].ALL_pathnode[i].time ;
                    time_delay_limit = time/time_sample;
                    break;
                }
            }
            p_state_ = P_STATE_TIME_DELAY;
            //p_state_ = P_STATE_MOVE;

        break;

        case MISSON_Virtual_traffic:
            std::cout<<"MISSON_Virtual_traffic" <<std::endl;
            Command_STATE = Command_STATE_VirtualTraffic;
            //停在交管點上 使它前往下一個任務
            if(trafficGO_recv.id==id)
                if(trafficGO_recv.GO ==1)
                {
                    p_state_ = P_STATE_MOVE;
                    trafficGO_recv.GO = 0 ;
                   memset(&trafficGO_recv,0,sizeof(trafficGO_recv));
                   std::cout<<"===State_Machine MISSON_Virtual_traffic==="<<std::endl;
                }

            //重派 時間到發送


            cnt_virtual_traffic_limit = reMission_time/time_sample ;
            if(cnt_virtual_traffic < cnt_virtual_traffic_limit)
                cnt_virtual_traffic++;
            else
            {
                cnt_virtual_traffic = 0;
                std_msgs::Int8 msg;

                msg.data = 1;

                ReMissionState_Publisher_.publish(msg);
            }


            //如果交管為最後 依然可以轉換為IDLE （配合交管使用）
            if(ready_path_index >= A_misson.size())
            {
                ready_path_index =0;
                A_misson[ready_path_index].ALL_pathnode.clear();
                A_misson.clear();
                p_state_=P_STATE_IDLE;
                isALLfinish =false;
				ChangToTraffic_finishstop = false;
                ROS_INFO("All Success!!!\n");

            }

        break;


        case MISSON_Elevator_Entrance:
            std::cout<<"MISSON_Elevator_Entrance" <<std::endl;
            Command_STATE = Command_Elevator_Entrance;

						//Change_Map_name = A_misson[now_A_misson].sub_missonPath[now_path_index + 2].map;
						std::cout<<"send_speak "<<Command_STATE <<std::endl;
						if(!send_speak)
						{
								std::cout<<"SPEAK send" <<std::endl;

								std_msgs::String speak_msg;
								speak_msg.data = "0";
								Speak_Publisher_.publish(speak_msg);
								send_speak = true;

						}

            //停在電梯前等待開門
            if(ElevatorGO)
            {
                p_state_ = P_STATE_MOVE;
                std::cout<<"===State_Machine MISSON_Elevator_Entrance==="<<ElevatorGO<<std::endl;
								//send_changemap = false;
								send_speak = false;
            }

        break;


        case MISSON_Elevator_Inside:
            std::cout<<"MISSON_Elevator_Inside" <<std::endl;
            Command_STATE = Command_Elevator_Inside;

            Change_Map_name = A_misson[now_A_misson].sub_missonPath[now_path_index + 1].map;
            ini_pose =  A_misson[now_A_misson].sub_missonPath[now_path_index + 1].sub_missonPath_subPoint[0];

            if(!send_changemap)
            {
							std::cout<<"send_changemap" <<std::endl;
                std::string command_str ="Load Change Map";
                move_robot::setmap_ctr msg;
                msg.type = command_str;
                msg.Name = Change_Map_name;
                msg.ini_pose_x = ini_pose.x();
                msg.ini_pose_y = ini_pose.y();
                msg.ini_pose_z = ini_pose.z();
                Command_Publisher_.publish(msg);

                send_changemap = true;

            }

						if(!send_speak)
					 {
							 std::cout<<"SPEAK send" <<std::endl;

							 std::string say_floor = std::to_string(A_misson[now_A_misson].sub_missonPath[now_path_index].floor);
							 std::cout<<"say_floor"<< say_floor <<std::endl;

							 std_msgs::String speak_msg;
							 speak_msg.data = say_floor;
							 Speak_Publisher_.publish(speak_msg);
							 send_speak = true;

					 }

			if(!changemap_finish)
				std::cout<<"********************"<<std::endl;

			if(changemap_finish)
			if(A_misson[now_A_misson].sub_missonPath[now_path_index].floor == floor)
			{
                if(ElevatorGO)
                {
                    p_state_ = P_STATE_MOVE;
                    std::cout<<"===State_Machine MISSON_Elevator_Inside=== GO"<<ElevatorGO<<std::endl;
										send_changemap = false;
										send_speak = false;
                }

			}

        break;


        case MISSON_Samefloor_ChangeMap:
            std::cout<<"MISSON_Samefloor_ChangeMap" <<std::endl;
            Command_STATE = Command_Samefloor_ChangeMap;

            Change_Map_name = A_misson[now_A_misson].sub_missonPath[now_path_index + 1].map;
            ini_pose =  A_misson[now_A_misson].sub_missonPath[now_path_index + 1].sub_missonPath_subPoint[0];

						if(!send_changemap)
            {
						std::cout<<"send_changemap" <<std::endl;
            std::string command_str ="Load Change Map";
            move_robot::setmap_ctr msg;
            msg.type = command_str;
            msg.Name = Change_Map_name;
            msg.ini_pose_x = ini_pose.x();
            msg.ini_pose_y = ini_pose.y();
            msg.ini_pose_z = ini_pose.z();
            Command_Publisher_.publish(msg);

            send_changemap = true;
			//std::cout<<"ini_pose  " <<ini_pose<<std::endl;

						std::cout<<"==send_changemap==" <<std::endl;
						}

					if(changemap_finish)
					{
						p_state_ = P_STATE_MOVE;
						send_changemap = false;
					}


            // time_delay_limit = 2.0/time_sample;
            // p_state_ = P_STATE_TIME_DELAY;


        break;


    }
	}
	else
	{
		send_speak = false;
		send_changemap = false;
		cnt_traffic = 0;
		cnt_virtual_traffic = 0;
	}



}

int Move_Robot::calc_target_index(Eigen::Vector3f &robot_pose, float &robot_v, std::vector<Eigen::Vector3f> &subpath, int &now_index)
{

    float min_error_value = 10000000;
    int min_index = -1;
    //當前路徑所有點與車子計算距離取最小
    for(int i=0; i<subpath.size(); i++){
        Eigen::Vector3f sub_goal = subpath[i];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        float error = sqrt(dx*dx + dy*dy);
        if(error < min_error_value){
            min_error_value = error;
            min_index = i;
        }
    }

    int index = min_index;
    now_index = index;

    float k = target_index_V_ratio;
    float Lfc = target_index_distance;

    float Lf = k*robot_v + Lfc;
		//float Lf = 0.4;
    float L = 0;

    //依速度調整前世距離的點
    while(Lf > L && (min_index+1) < (subpath.size())){
        Eigen::Vector3f sub_goal = subpath[min_index + 1];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        L = sqrt(dx*dx + dy*dy);
        min_index += 1;
    }
    return min_index;
}

// 預測2公尺處轉彎用 轉彎減速
int Move_Robot::calc_target_index_obsturn(Eigen::Vector3f &robot_pose, float &robot_v, std::vector<Eigen::Vector3f> &subpath, int &now_index)
{

    float min_error_value = 10000000;
    int min_index = -1;
    //當前路徑所有點與車子計算距離取最小
    for(int i=0; i<subpath.size(); i++){
        Eigen::Vector3f sub_goal = subpath[i];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        float error = sqrt(dx*dx + dy*dy);
        if(error < min_error_value){
            min_error_value = error;
            min_index = i;
        }
    }

    int index = min_index;
    now_index = index;

    // float k = 0.5;
    // //float Lfc = k*robot_v + 2.0;
		// float Lfc = 1.1;
    //float Lf = Lfc;

		float k = 1.0;
    float Lfc = 0.4;

    //float Lf = (k*0.6 + Lfc)*1.5;
		float Lf = 3.0;
    float L = 0;

    //依速度調整前世距離的點
    while(Lf > L && (min_index+1) < (subpath.size())){
        Eigen::Vector3f sub_goal = subpath[min_index + 1];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        L = sqrt(dx*dx + dy*dy);
        min_index += 1;
    }
    return min_index;
}


int Move_Robot::calc_target_index_obs(Eigen::Vector3f &robot_pose, float &robot_v, std::vector<Eigen::Vector3f> &subpath, int &now_index)
{

    float min_error_value = 10000000;
    int min_index = -1;

    for(int i=0; i<subpath.size(); i++){
        Eigen::Vector3f sub_goal = subpath[i];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        float error = sqrt(dx*dx + dy*dy);
        if(error < min_error_value){
            min_error_value = error;
            min_index = i;
        }
    }

    int index = min_index;
    now_index = index;

    float k = 0.35;
    //=================change=============
    float Lfc = 0.4;

    //float Lf = k*robot_v + Lfc;
		float Lf = 0.4;
    float L = 0;

    while(Lf > L && (min_index+1) < (subpath.size())){
        Eigen::Vector3f sub_goal = subpath[min_index + 1];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        L = sqrt(dx*dx + dy*dy);
        min_index += 1;
    }
    return min_index;
}

int Move_Robot::calc_target_index_key(Eigen::Vector3f &robot_pose, float &dis, std::vector<Eigen::Vector3f> &subpath, int &now_index)
{
    float min_error_value = 10000000;
    int min_index = -1;

    for(int i=0; i<subpath.size(); i++){
        Eigen::Vector3f sub_goal = subpath[i];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        float error = sqrt(dx*dx + dy*dy);
        if(error < min_error_value){
            min_error_value = error;
            min_index = i;
        }
    }

    int index = min_index;
    now_index = index;


    float Lf = dis;
    float L = 0;

    while(Lf > L && (min_index+1) < (subpath.size())){
        Eigen::Vector3f sub_goal = subpath[min_index + 1];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        L = sqrt(dx*dx + dy*dy);
        min_index += 1;
    }
    return min_index;
}
int Move_Robot::calc_target_index_MPC(Eigen::Vector3f &robot_pose,  std::vector<Eigen::Vector3f> &subpath, int &now_index)
{
    float min_error_value = 10000000;
    int min_index = -1;

    for(int i=0; i<subpath.size(); i++){
        Eigen::Vector3f sub_goal = subpath[i];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        float error = sqrt(dx*dx + dy*dy);
        if(error < min_error_value){
            min_error_value = error;
            min_index = i;
        }
    }

    int index = min_index;
    now_index = index;


    //float Lf = 2 + CarLength_helf;
		float Lf = 0.83;
    float L = 0;

    while(Lf > L && (min_index+1) < (subpath.size())){
        Eigen::Vector3f sub_goal = subpath[min_index + 1];
        float dx = robot_pose.x() - sub_goal.x();
        float dy = robot_pose.y() - sub_goal.y();
        L = sqrt(dx*dx + dy*dy);
        min_index += 1;
    }
    return min_index;
}


void Move_Robot::MissMsgCallback(const std_msgs::Int16& msg)
{

    int isMissing = msg.data;
    //ROS_INFO("  isMissing   ");

    if(isMissing && !isReveice_joystick){
        //Stop();
        p_state_ = P_STATE_STOP;
        ROS_INFO("  isMissing   ");
        v_stop = true;
    }
    else
    {
        if(v_stop)
	        if(A_misson.size() > 0)
            {
		        if(time_delay_counter > 0)
			        {
				        p_state_ = P_STATE_TIME_DELAY;
				        std::cout<<"GO_Loading"<<std::endl;
			        }
		        else
			    {
				    p_state_ = P_STATE_MOVE;
				    std::cout<<"GO_move"<<std::endl;
			    }
            }
	        else
		        p_state_ = P_STATE_IDLE;
        v_stop = false;
    }



}
void Move_Robot::slamoutCallback(const geometry_msgs::PoseStamped& msg)
{
    tf::Pose pose_tf;
    tf::poseMsgToTF(msg.pose, pose_tf);
    slam_pose_ = Eigen::Vector3f(msg.pose.position.x, msg.pose.position.y, tf::getYaw(pose_tf.getRotation()));
    //ROS_INFO("Robot pose x: %f y: %f yaw: %f", slam_pose_[0], slam_pose_[1], slam_pose_[2]);

    move_robot::state msg1;
    msg1.id = Command_id;
    msg1.state = Command_STATE;
    STATE_Publisher_.publish(msg1);

}
void Move_Robot::UKFCallback(const geometry_msgs::PoseStamped& msg)
{
    tf::Pose pose_tf;
    tf::poseMsgToTF(msg.pose, pose_tf);
    ukf_pose_ = Eigen::Vector3f(msg.pose.position.x, msg.pose.position.y, tf::getYaw(pose_tf.getRotation()));
}



void Move_Robot::SendPackage(std::vector<unsigned char> recv)
{

    unsigned char command[recv.size()];
    for(int i = 0;i<recv.size(); i++)
	{
            command[i] = recv[i];
//            printf("%hhx \n",command[i]);
	}

    int nByte = 0;
    if(mySerial.serial_ok)
       nByte = write(mySerial.fd,command,recv.size());


}

void Move_Robot::SendPackage_two(double cmd_vl, double cmd_vr)
{

     //std::cout<<"-----------"<<std::endl;
    // std::cout<<"vl: "<<cmd_vl<<std::endl;
     //std::cout<<"vr: "<<cmd_vr<<std::endl;

    double send_vl = cmd_vl + 20.0;
    double send_vr = cmd_vr + 100.0;

    unsigned char command[13];

    //std::cout<<"send_vl: "<<send_vl<<std::endl;
    //std::cout<<"send_vr: "<<send_vr<<std::endl;



    int integer_vl = (int(send_vl));
    int float_vl = ( int((send_vl - double(integer_vl))*1000 ));
    int integer_vr = (int(send_vr));
    int float_vr = ( int((send_vr - double(integer_vr))*1000 ));


    int HighByte_integer_vl = integer_vl/128;
    int LowByte_integer_vl  = integer_vl%128;
    int HighByte_float_vl   = float_vl/128;
    int LowByte_float_vl    = float_vl%128;

    int HighByte_integer_vr = integer_vr/128;
    int LowByte_integer_vr  = integer_vr%128;
    int HighByte_float_vr   = float_vr/128;
    int LowByte_float_vr    = float_vr%128;


    command[ 0] = 'S';
    command[ 1] = 'A';
    command[ 2] = HighByte_integer_vl;
    command[ 3] = LowByte_integer_vl;
    command[ 4] = HighByte_float_vl;
    command[ 5] = LowByte_float_vl;
    command[ 6] = HighByte_integer_vr;
    command[ 7] = LowByte_integer_vr;
    command[ 8] = HighByte_float_vr;
    command[ 9] = LowByte_float_vr;
    command[10] = 'E';
    command[11] = 'N';
    command[12] = 'D';

    for(int i = 0;i<1000000;i++)
    {
        //std::cout<<i<<std::endl;
    }


    int nByte = 0;
    if(mySerial.serial_ok){
       nByte = write(mySerial.fd,command,13);
       ROS_INFO("AAA  %d", nByte);
       std::cout<<"command  "<<command[0]<<std::endl;
    }
}

void Move_Robot::NormalizeAngle(float& phi)
{
    phi = atan2(sin(phi), cos(phi));
}
void Move_Robot::draw(int id,float color_r,float color_g,float color_b,std::vector<Eigen::Vector3f> drawpoint)
{
    //畫出軌跡再rviz
            visualization_msgs::Marker points;
            points.header.frame_id  = "/map";
            points.header.stamp  = ros::Time::now();
            points.ns  = "points";
            points.action  = visualization_msgs::Marker::ADD;
            points.pose.orientation.w  = 1.0;
            points.id = id;


            points.type = visualization_msgs::Marker::POINTS;

            // POINTS markers use x and y scale for width/height respectively
            points.scale.x = 0.05;
            points.scale.y = 0.05;

            // Points are green
            points.color.g = color_g;
            points.color.b = color_b;
            points.color.r = color_r;
            points.color.a = 1.0;

            // Create the vertices for the points and lines
            for(int i=0;i<drawpoint.size() ;i++)
            {

            geometry_msgs::Point p;
            p.x = drawpoint[i].x();
            p.y = drawpoint[i].y();
            p.z = 0;

            points.points.push_back(p);

            }

            // std::cout<<"sub_missonPath_subPoint_buf.sub_missonPath_subPoint.size()  "<<sub_missonPath_subPoint_buf.sub_missonPath_subPoint.size()<<std::endl;
            // std::cout<<"points  "<<points.points.size()<<std::endl;
            //std::cout<<"draw  "<<std::endl;
            marke_Publisher_.publish(points);
            //畫出軌跡再rviz
}
void Move_Robot::drawLine(int id,float color_r,float color_g,float color_b,std::vector<Eigen::Vector3f> drawline)
{
    //std::cout<<"========drawLine========"<<std::endl;
    //畫出軌跡再rviz
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w =1.0;
    line_strip.id = id;



    line_strip.type = visualization_msgs::Marker::LINE_STRIP;


    line_strip.scale.x = 0.1;

    line_strip.color.r = color_r;
    line_strip.color.g = color_g;
    line_strip.color.b = color_b;
    line_strip.color.a = 1.0;


 for(int i=0;i<drawline.size() ;i++)
    {

        geometry_msgs::Point line;
        line.x = drawline[i].x();
        line.y = drawline[i].y();
        line.z = drawline[i].z();
        line_strip.points.push_back(line);

    }
    marke_Publisher_.publish(line_strip);

    //畫出軌跡再rviz
}
int Move_Robot::SetLocalpar(std::string file_buf)
{
    std::fstream fin;
    int Car_packMode = -1;

    char *file = const_cast<char *>(file_buf.c_str());
    fin.open(file, std::fstream::in);
    if(!fin.is_open())
    {
        ROS_INFO("Error: Localpar is not opened!!");
    }
    else{
        ROS_INFO("the file is opened!!");



        std::string kp;
        std::getline(fin, kp);
        std::getline(fin, kp);
        tracking_kp = std::atof(kp.c_str());

        std::string kd;
        std::getline(fin, kd);
        std::getline(fin, kd);
        tracking_kd = std::atof(kd.c_str());

        std::string s_Car_packMode;
        std::getline(fin, s_Car_packMode);
        std::getline(fin, s_Car_packMode);
        // Car_packMode = s_Car_packMode;

        if(s_Car_packMode == "ChinaMotor")
            Car_packMode = 0;
        else if(s_Car_packMode =="IVAM_Car_vlvr")
            Car_packMode = 0;
        else if(s_Car_packMode =="IVAM_Car_vw")
            Car_packMode = 1;
        else if(s_Car_packMode =="Boling_smallgray")
            Car_packMode = 2;
				else if(s_Car_packMode =="public_wheel")
		        Car_packMode = 0;

				std::string ChangeMode_dis_error_temp;
				std::getline(fin, ChangeMode_dis_error_temp);
		    std::getline(fin, ChangeMode_dis_error_temp);
				ChangeMode_dis_error = std::atof(ChangeMode_dis_error_temp.c_str());

				std::string ChangeMode_angular_error_temp;
				std::getline(fin, ChangeMode_angular_error_temp);
		    std::getline(fin, ChangeMode_angular_error_temp);
				ChangeMode_angular_error = std::atof(ChangeMode_angular_error_temp.c_str());

				std::string traffic_dis_error_temp;
				std::getline(fin, traffic_dis_error_temp);
		    std::getline(fin, traffic_dis_error_temp);
			  traffic_dis_error = std::atof(traffic_dis_error_temp.c_str());

				std::string othermode_dis_error_temp;
				std::getline(fin, othermode_dis_error_temp);
		    std::getline(fin, othermode_dis_error_temp);
				othermode_dis_error = std::atof(othermode_dis_error_temp.c_str());

				std::string othermode_angular_error_temp;
				std::getline(fin, othermode_angular_error_temp);
		    std::getline(fin, othermode_angular_error_temp);
				othermode_angular_error = std::atof(othermode_angular_error_temp.c_str());

				std::string target_index_V_ratio_temp;
				std::getline(fin, target_index_V_ratio_temp);
		    std::getline(fin, target_index_V_ratio_temp);
				target_index_V_ratio = std::atof(target_index_V_ratio_temp.c_str());

				std::string target_index_distance_temp;
				std::getline(fin, target_index_distance_temp);
		    std::getline(fin, target_index_distance_temp);
				target_index_distance = std::atof(target_index_distance_temp.c_str());

				std::string network_card_temp;
				std::getline(fin, network_card_temp);
		    std::getline(fin, network_card_temp);
				network_card = network_card_temp;
    }

    fin.close();

    return Car_packMode;
}
double Move_Robot::polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

Eigen::VectorXd Move_Robot::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
