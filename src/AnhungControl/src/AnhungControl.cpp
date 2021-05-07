#include "AnhungControl.h"
#include <time.h>

Control::Control(char *address, char *SRV_IP, int port)
{
	IP = SRV_IP;
	Port = port;
	begin_time = ros::Time::now ();

	LoadTitlePath();
	LocalparPATH = TitlePath + LocalparPATH_Local;
	//std::cout<<"LocalparPATH "<<LocalparPATH<<std::endl;
  SetLocalpar(LocalparPATH);
	SearchIP();

	setTime();
	CarParameterPATH = TitlePath + CarParameterPATH_Local;

	CarName = "NULL";
	Carnumber = int2str(-1);



	registration = 0;

	//joystick
	btn_id = 0;
	isReveice_joystick = false;
	protect_SendNode = false;



	isMissing = 0;
	Command_id = 0;
	Command_STATE = 0;

	V_now = 0;

	argv = address;

	path.clear();

	isSetMapTrans = false;

	isLoadMap = false;
	//建地圖封包
	commmand_buf = -1;
	//連線測試封包
	Connect_buf = "";

	//重派封包
	Start_ID = -1;
	End_ID = -1;
	End_x = 0.0;
	End_y = 0.0;
	End_z = 0.0;





	//廣播
	// setsockopt(sfd_,SOL_SOCKET,SO_BROADCAST,&si_BROADCAST,sizeof(si_BROADCAST));
	// memset((char *) &si_BROADCAST, 0, sizeof(si_BROADCAST));
	// si_BROADCAST.sin_family = AF_INET; /*IPv4*/
	// si_BROADCAST.sin_port = htons(port);  /*Set port number*/
	// si_BROADCAST.sin_addr.s_addr = htonl(INADDR_BROADCAST); /*The broadcast address*/
	// bind(sfd_, (struct sockaddr *)&si_BROADCAST, sizeof(si_BROADCAST));
	// clientlen = sizeof(si_BROADCAST);
	//sendto(sfd_, all_data, std::strlen(all_data), 0, (struct sockaddr*)&si_BROADCAST, (socklen_t)clientlen);

	imagePATH = TitlePath + imagePATH_Local  ;
	char *imagePATH_buf = const_cast<char *>(imagePATH.c_str());

	Mapname = Lc_par.map_name;
	QImage map_image;
	map_image = QImage(imagePATH_buf);


	max_map_width = map_image.width();
	max_map_height = map_image.height();
	m_resolution = float(50.0)/1000;
	Eigen::Vector2f StartCoord(float(max_map_width)*0.50*m_resolution , float(max_map_height)*0.50*m_resolution);
	setMapTransformation(StartCoord, m_resolution);

	LoadCarParameter(CarParameterPATH);


	traffic_recv_buf.id = 0;
	traffic_recv_buf.GO = 0;
	traffic_recv_buf.speed = 0;
	SendNode = false;

	all_missiontime = 0;
	missiontime = 0;



	if(isSocketOpen_){
		IdType_Publisher_ = nodeHandle_.advertise<AnhungControl::Node_recv>("id",10000);
		trafficGO_Publisher_ = nodeHandle_.advertise<AnhungControl::traffic_recv>("GO",10);
		Clear_ObsMode_Publisher_ = nodeHandle_.advertise<std_msgs::Int8>("Clear_ObsMode",10);
		Trigger_Publisher_ = nodeHandle_.advertise<std_msgs::Int8>("Trigger",10);
		Command_Publisher_ = nodeHandle_.advertise<AnhungControl::setmap_ctr>("Command",10);
		floor_Publisher_ = nodeHandle_.advertise<std_msgs::Int8>("floor",10);


		ErrorSubscriber_=nodeHandle_.subscribe("Error", 5, &Control::ErrorCallback, this);
		map_Subscriber_ = nodeHandle_.subscribe("map", 1, &Control::mapCallback, this);
		slamoutSubscriber_ = nodeHandle_.subscribe("slam_out_pose", 5, &Control::slamoutCallback, this);
		//slamoutSubscriber_ = nodeHandle_.subscribe("fusionUKF", 5, &Control::slamoutCallback, this);
		MissMsgSubscriber_ = nodeHandle_.subscribe("Missing", 5, &Control::MissMsgCallback, this);
		STATESubscriber_ = nodeHandle_.subscribe("STATE", 5, &Control::STATECallback, this);
		JoysickSubscriber_ = nodeHandle_.subscribe("joystick", 5, &Control::joystickCallback, this);
		vSubscriber_ = nodeHandle_.subscribe("v", 5, &Control::vCallback, this);
		ReMissionStateSubscriber_ = nodeHandle_.subscribe("ReMissionState", 5, &Control::ReMissionStateCallback, this);
		BatterySubscriber_ = nodeHandle_.subscribe("Battery", 5, &Control::BatteryCallback, this);
		MapnameSubscriber_ = nodeHandle_.subscribe("Mapname", 5, &Control::MapnameCallback, this);


		recv_command_thread_ = new boost::thread(boost::bind(&Control::RevProcess, this, 0.05));
	}

	Opensend();
	sendScale();


}

Control::~Control()
{
	close(sfd_);
	//break;
	isSocketOpen_ = false;

	delete recv_command_thread_;
	std::cout<<"close class"<<std::endl;

}

void Control::SetLocalpar(std::string file_buf)
{
    std::fstream fin;
    int Car_packMode = -1;

		//std::cout<<"A "<<file_buf<<std::endl;

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
        Lc_par.tracking_kp = std::atof(kp.c_str());

        std::string kd;
        std::getline(fin, kd);
        std::getline(fin, kd);
        Lc_par.tracking_kd = std::atof(kd.c_str());

        std::string s_Car_packMode;
        std::getline(fin, s_Car_packMode);
        std::getline(fin, s_Car_packMode);
        // Car_packMode = s_Car_packMode;

        if(s_Car_packMode == "ChinaMotor")
            Lc_par.Car_packMode = 0;
        else if(s_Car_packMode =="IVAM_Car_vlvr")
            Lc_par.Car_packMode = 0;
        else if(s_Car_packMode =="IVAM_Car_vw")
            Lc_par.Car_packMode = 1;
        else if(s_Car_packMode =="Boling_smallgray")
            Lc_par.Car_packMode = 2;

				std::string ChangeMode_dis_error_temp;
				std::getline(fin, ChangeMode_dis_error_temp);
        std::getline(fin, ChangeMode_dis_error_temp);
				Lc_par.ChangeMode_dis_error = std::atof(ChangeMode_dis_error_temp.c_str());

				std::string ChangeMode_angular_error_temp;
				std::getline(fin, ChangeMode_angular_error_temp);
        std::getline(fin, ChangeMode_angular_error_temp);
				Lc_par.ChangeMode_angular_error = std::atof(ChangeMode_angular_error_temp.c_str());

				std::string traffic_dis_error_temp;
				std::getline(fin, traffic_dis_error_temp);
        std::getline(fin, traffic_dis_error_temp);
				Lc_par.traffic_dis_error = std::atof(traffic_dis_error_temp.c_str());

				std::string othermode_dis_error_temp;
				std::getline(fin, othermode_dis_error_temp);
        std::getline(fin, othermode_dis_error_temp);
				Lc_par.othermode_dis_error = std::atof(othermode_dis_error_temp.c_str());

				std::string othermode_angular_error_temp;
				std::getline(fin, othermode_angular_error_temp);
        std::getline(fin, othermode_angular_error_temp);
				Lc_par.othermode_angular_error = std::atof(othermode_angular_error_temp.c_str());

				std::string target_index_V_ratio_temp;
				std::getline(fin, target_index_V_ratio_temp);
        std::getline(fin, target_index_V_ratio_temp);
				Lc_par.target_index_V_ratio = std::atof(target_index_V_ratio_temp.c_str());

				std::string target_index_distance_temp;
				std::getline(fin, target_index_distance_temp);
        std::getline(fin, target_index_distance_temp);
				Lc_par.target_index_distance = std::atof(target_index_distance_temp.c_str());

				std::string network_card_temp;
				std::getline(fin, network_card_temp);
        std::getline(fin, network_card_temp);
				Lc_par.network_card = network_card_temp;

				std::string map_name_temp;
				std::getline(fin, map_name_temp);
        std::getline(fin, map_name_temp);
				Lc_par.map_name = map_name_temp;
    }

    fin.close();

}

void Control::LoadCarParameter(std::string file_buf)
{

	std::fstream fin;

	char *file = const_cast<char *>(file_buf.c_str());
	fin.open(file, std::fstream::in);
	if(!fin.is_open())
	{
		ROS_INFO("Error: you need send the CarParameter!!");
		ErrorState(0);
	}
	else{
		ROS_INFO("the file is opened!!");

		std::string s_CarName;
		std::getline(fin, s_CarName);
		std::getline(fin, s_CarName);
		par.CarName_ = s_CarName;

		std::string s_Carnumber;
		std::getline(fin, s_Carnumber);
		std::getline(fin, s_Carnumber);
		par.Carnumber = std::atoi(s_Carnumber.c_str());

		std::string s_CarKind;
		std::getline(fin, s_CarKind);
		std::getline(fin, s_CarKind);
		par.CarKind = std::atoi(s_CarKind.c_str());

		std::string s_CarLength;
		std::getline(fin, s_CarLength);
		std::getline(fin, s_CarLength);
		par.CarLength = std::atof(s_CarLength.c_str());

		std::string s_CarWidth;
		std::getline(fin, s_CarWidth);
		std::getline(fin, s_CarWidth);
		par.CarWidth = std::atof(s_CarWidth.c_str());

		std::string s_LRWheeldis;
		std::getline(fin, s_LRWheeldis);
		std::getline(fin, s_LRWheeldis);
		par.LRWheeldis = std::atof(s_LRWheeldis.c_str());

		std::string s_FBWheeldis;
		std::getline(fin, s_FBWheeldis);
		std::getline(fin, s_FBWheeldis);
		par.FBWheeldis = std::atof(s_FBWheeldis.c_str());

		std::string s_LWheeldis;
		std::getline(fin, s_LWheeldis);
		std::getline(fin, s_LWheeldis);
		par.LWheeldis = std::atof(s_LWheeldis.c_str());

		std::string s_Wheel_r;
		std::getline(fin, s_Wheel_r);
		std::getline(fin, s_Wheel_r);
		par.Wheel_r = std::atof(s_Wheel_r.c_str());

		std::string s_Reduction_Ratio;
		std::getline(fin, s_Reduction_Ratio);
		std::getline(fin, s_Reduction_Ratio);
		par.Reduction_Ratio = std::atof(s_Reduction_Ratio.c_str());

		std::string s_speedup;
		std::getline(fin, s_speedup);
		std::getline(fin, s_speedup);
		par.speedup = std::atof(s_speedup.c_str());

		std::string s_speed;
		std::getline(fin, s_speed);
		std::getline(fin, s_speed);
		par.speed = std::atof(s_speed.c_str());

		std::string s_JOYSTICK_SCALAR;
		std::getline(fin, s_JOYSTICK_SCALAR);
		std::getline(fin, s_JOYSTICK_SCALAR);
		par.JOYSTICK_SCALAR = std::atof(s_JOYSTICK_SCALAR.c_str());


		//OBS

		std::string s_OBS_Front;
		std::getline(fin, s_OBS_Front);
		std::getline(fin, s_OBS_Front);
		//OBS_Front = std::atof(s_OBS_Front.c_str());

		std::string s_OBS_Side;
		std::getline(fin, s_OBS_Side);
		std::getline(fin, s_OBS_Side);
		//OBS_Side = std::atof(s_OBS_Side.c_str());

		std::string s_OBS_Front_limit;
		std::getline(fin, s_OBS_Front_limit);
		std::getline(fin, s_OBS_Front_limit);
		//OBS_Front_limit = std::atof(s_OBS_Front_limit.c_str());

		std::string s_OBS_Stop_Sec;
		std::getline(fin, s_OBS_Stop_Sec);
		std::getline(fin, s_OBS_Stop_Sec);
		//OBS_Stop_Sec = std::atof(s_OBS_Stop_Sec.c_str());

		std::string s_OBS_Wait_Sec;
		std::getline(fin, s_OBS_Wait_Sec);
		std::getline(fin, s_OBS_Wait_Sec);
		//OBS_Wait_Sec = std::atof(s_OBS_Wait_Sec.c_str());

		std::string s_OBS_Theta_Sec;
		std::getline(fin, s_OBS_Theta_Sec);
		std::getline(fin, s_OBS_Theta_Sec);
		//OBS_Theta_Sec = std::atof(s_OBS_Theta_Sec.c_str());

		//traffic
		std::string s_traffic_dis;
		std::getline(fin, s_traffic_dis);
		std::getline(fin, s_traffic_dis);
		par.traffic_dis = std::atof(s_traffic_dis.c_str());

		std::string s_reMission_time;
		std::getline(fin, s_reMission_time);
		std::getline(fin, s_reMission_time);
		par.reMission_time = std::atof(s_reMission_time.c_str());




		//        std::cout<<"s_CarWidth  "<<CarWidth<<std::endl;
		//        std::cout<<"LRWheeldis  "<<LRWheeldis<<std::endl;
		//        std::cout<<"FBWheeldis  "<<FBWheeldis<<std::endl;
		//        std::cout<<"LWheeldis  "<<LWheeldis<<std::endl;
		//        std::cout<<"Wheel_r  "<<Wheel_r<<std::endl;
		//        std::cout<<"Reduction_Ratio  "<<Reduction_Ratio<<std::endl;
		//        std::cout<<"speedup  "<<speedup<<std::endl;
		// std::cout<<"par.traffic_dis  "<<par.traffic_dis<<std::endl;
		// std::cout<<"par.reMission_time  "<<par.reMission_time<<std::endl;

		registration = 1 ;

		CarName = par.CarName_;
		Carnumber = int2str(par.Carnumber);

	}

	fin.close();




}

void Control::RevProcess(double receive_period)
{

	ros::Rate r(1.0 / receive_period);
	ros::Rate reconnect(1.0 / 2.0);


	//任務鏈封包
	bool isMisson_pixel = false;         //Mp E
	bool isMisson_real = false;         //Mr E

	//交管點封包
	bool isTraffic = false;       //T E
	//刪掉目前的任務與避障模式選擇
	bool isClear_ObsMode_Misson = false;   //C E
	//設置車子參數
	bool SetParameter = false; //P
	//設置Localparameter
	bool SetLoaclParameter = false; //P
	//建地圖 存地圖 nav
	bool SetMap = false; //Sm
	//連線測試
	bool ConnectTest = false; //I

	//交管重派封包
	bool RpMisson_pixel = false;         //Rp E
	bool RrMisson_real = false;         //Rr E
	//中華汽車跨樓層用
	bool Confirm_floor = false;    //F E

	int clear_buf=0;
	int cnt_protect = 0;
	std::string s_Car_packMode = " ";

	//std::cout<<"RevProcess"<<std::endl;

	while(ros::ok())
	{

		//socket initial
		isSocketOpen_ = true;
		slen_=sizeof(si_other_);

		if ((sfd_=socket(AF_INET, SOCK_STREAM, 0))==-1){
			ROS_INFO("Socket Error");
			isSocketOpen_ = false;
		}

		char *local_IP = const_cast<char*>(IP_buf.c_str());

		si_me_.sin_family = AF_INET;
		si_me_.sin_addr.s_addr=inet_addr(local_IP);//服务器IP地址--允许连接到所有本地地址上
		si_me_.sin_port=0; //服务器端口号
		bind(sfd_,(struct sockaddr *)&si_me_,sizeof(si_me_));

		memset(&si_other_,0,sizeof(si_other_)); //数据初始化--清零
		si_other_.sin_family=PF_INET; //设置为IP通信
		//si_me_.sin_addr.s_addr=INADDR_ANY;//服务器IP地址--允许连接到所有本地地址上
		si_other_.sin_addr.s_addr=inet_addr(IP);//服务器IP地址--允许连接到所有本地地址上
		si_other_.sin_port=htons(Port); //服务器端口号

		struct timeval timeo= {3, 0};
		socklen_t tim_len = sizeof(timeo);

		setsockopt(sfd_, SOL_SOCKET, SO_SNDTIMEO,&timeo, tim_len);


		int err = connect(sfd_,(struct sockaddr *)&si_other_,sizeof(si_other_));
		//std::cout<<"RevProcess "<< err <<std::endl;
		if(err==-1){
				if (errno== EINPROGRESS){
					isSocketOpen_ = false;
					fprintf(stderr,"timeout\n");
				}
				isSocketOpen_ = false;
				//printf("Connection error\n");
				ROS_ERROR("Connection error\n");
				//ROS_ERROR("我\n");

				reconnect.sleep();
		}


		while(isSocketOpen_){

			char buf[5000]={0};
			std::string cut_par;
			std::string recv_pkg[100];
			std::string recv_pkg_subitem[100];

			int end=0;

			int n = recvtimeout(sfd_, buf, 5000, 50000);

			if(n == -1){
				//std::cout<<"Receive Error!!!"<<std::endl;
			}
			else if(n == -2){
				//std::cout<<"Timeout"<<std::endl;
			}
			else{

				if(n == 0)
				{
						std::cout<<"client disconnect"<<std::endl;
						close(sfd_);
						//break;
						isSocketOpen_ = false;
				}
				else
				{
						//近來就先拆開
						int count=0;
						std::cout<<" buf= "<<buf<<std::endl;
						std::stringstream cut(buf);
						while(getline(cut,cut_par,';'))
						{
							recv_pkg[count]=cut_par;
							std::cout<<" recv_pkg= "<<count << " "<< recv_pkg[count]<<std::endl;
							count++;
						}

						end=count-1;

						bool pkg_wrong = false;
						count=0;
						int count1=0;
						while(!recv_pkg[count].empty())
						{
							NODE_recv node_buf;



							//近來就先拆開，就能檢查封頭
							std::stringstream cut(recv_pkg[count]);
							while(getline(cut,cut_par,','))
							{

								recv_pkg_subitem[count1]=cut_par;
								//std::cout<<" count= "<<count << " end"<< end<<std::endl;


								//===========檢查封頭=======
								if(recv_pkg_subitem[0]=="Mp"&&count==0)
								{
									count1++;
									isMisson_pixel=true;
								}
								else if(recv_pkg_subitem[0]=="Mr"&&count==0)
								{
									count1++;
									isMisson_real=true;
								}
								else if(recv_pkg_subitem[0]=="T"&&count==0)
								{
									count1++;
									isTraffic=true;
								}
								else if(recv_pkg_subitem[0]=="C"&&count==0)
								{
									count1++;
									isClear_ObsMode_Misson=true;
								}
								else if(recv_pkg_subitem[0]=="P"&&count==0)
								{
									count1++;
									SetParameter = true;
								}
								else if(recv_pkg_subitem[0]=="L"&&count==0)
								{
									count1++;
									SetLoaclParameter = true;
								}
								else if(recv_pkg_subitem[0]=="Sm"&&count==0)
								{
									count1++;
									SetMap = true;
								}
								else if(recv_pkg_subitem[0]=="I"&&count==0)
								{
									count1++;
									ConnectTest = true;
								}
								else if(recv_pkg_subitem[0]=="Rp"&&count==0)
								{
									count1++;
									RpMisson_pixel = true;
								}
								else if(recv_pkg_subitem[0]=="Rr"&&count==0)
								{
									count1++;
									RrMisson_real = true;
								}
								else if(recv_pkg_subitem[0]=="F"&&count==0)
								{
									count1++;
									Confirm_floor = true;
								}
								else
								{
									if(count==0)
									{
										pkg_wrong=true;
										break;
									}
								}

								//============如果是任務=============
								if(isMisson_pixel || isMisson_real)
								{

									if(count>0&& count<end)
									{
										std::istringstream item(recv_pkg_subitem[count1]);
										switch (count1){
											case 0:
												item>>node_buf.id;
												break;
											case 1:
												item>>node_buf.type;
												break;
											case 2:
												item>>node_buf.id_pixel_x;
												break;
											case 3:
												item>>node_buf.id_pixel_y;
												break;
											case 4:
												item>>node_buf.id_heading;
												break;
											case 5:
												item>>node_buf.kin;
												break;
											case 6:
												item>>node_buf.line;
												break;
											case 7:
												item>>node_buf.radius;
											case 8:
												item>>node_buf.map;
												break;
											default:
												if(recv_pkg_subitem[1]=="3" || recv_pkg_subitem[1]=="5" || recv_pkg_subitem[1]=="9") item>>node_buf.time;
												else if(recv_pkg_subitem[1]=="4" ||recv_pkg_subitem[1]=="6") item>>node_buf.btn_finish;
												else if(recv_pkg_subitem[1]=="12") item>>node_buf.floor;
												else {
													//std::cout<<"===========QQQQQQQQQQ============="<<std::endl;
													pkg_wrong=true;
													isMisson_pixel=false;
													isMisson_real=false;
													memset(&node_buf,0,sizeof(node_buf));
													break;
												}

												//type=0  C
												//type=1  P
												//type=2  T
												//type=3  LT
												//type=4  LI
												//type=5  UT
												//type=6  UI
												//type=7  modetra
												//type=8  臨時靠站點
												//type=9  FreeLoading
												//type=10  FreeLoading
												//type=11 電梯口
												//type=12 電梯內
												//type=13 同層切換點

												//line=0 Straight
												//line=1 Arc
												//line=2 circle

												break;
										}
										//std::cout<<" recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;


										count1++;
									}


									//===========檢查封尾=======
									if(recv_pkg_subitem[0]=="E"&&count==end)
									{
										std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
									}
									else if(recv_pkg_subitem[0]!="E"&&count==end)
									{
										//std::cout<<"===========AAAAAAAAAAAAAAA============="<<std::endl;
										pkg_wrong = true;
										isMisson_pixel = false;
										isMisson_real = false;
										memset(&node_buf,0,sizeof(node_buf));
										break;
									}
								}

								//==============如果是任務=============


								//==============如果是交管點封包=============

								if(isTraffic)
								{
									//delete [] node_buf;
									if(count>0&& count<end)
									{
										std::istringstream item(recv_pkg_subitem[count1]);
										switch (count1){
											case 0:
												item>>traffic_recv_buf.id;
												break;
											case 1:
												item>>traffic_recv_buf.GO;
												break;
											default:
												if(recv_pkg_subitem[1]=="3")
												{
													item>>traffic_recv_buf.speed;
												}
												break;
										}
										//std::cout<<" recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;

										cnt_protect = count1;
										count1++;
									}


									//===========檢查封尾=======
									if(recv_pkg_subitem[0]=="E"&&count==end)
									{
										std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
										if(cnt_protect > 2)
										{
											pkg_wrong=true;
											isTraffic=false;
											break;
										}
										else if(cnt_protect == 1)
										{
											traffic_recv_buf.speed = 0.0;
											break;
										}
									}
									else if(recv_pkg_subitem[0]!="E"&&count==end)
									{
										pkg_wrong=true;
										isTraffic=false;
										break;
									}
								}

								//==============如果是交管點封包=============


								//==============如果是clear封包=============

								if(isClear_ObsMode_Misson)
								{
									//delete [] node_buf;
									if(count>0&& count<end)
									{
										std::istringstream item(recv_pkg_subitem[count1]);

										item>> clear_buf;

										//std::cout<<" recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;


										count1++;
									}


									//===========檢查封尾=======
									if(recv_pkg_subitem[0]=="E"&&count==end)
									{
										std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
									}
									else if(recv_pkg_subitem[0]!="E"&&count==end)
									{
										pkg_wrong=true;
										isClear_ObsMode_Misson=false;
										break;
									}
								}

								//==============如果是clear_OBS封包=============

								//==============如果是車子參數封包=============
								if(SetParameter)
								{
									std::cout<<"count  "<< count<<std::endl;
									std::string debug_;
									if(count>0&& count<end)
									{
										std::istringstream item(recv_pkg_subitem[count1]);
										switch (count1){
											case 0:
												item>>debug_;
												par.CarName_ = debug_;
												//item>>par.CarName_; //不要問為什麼 我也不知道為什麼會出錯
												break;
											case 1:
												item>>par.Carnumber;
												break;
											case 2:
												item>>par.CarKind;
												break;
											case 3:
												item>>par.CarLength;
												break;
											case 4:
												item>>par.CarWidth;
												break;
											case 5:
												item>>par.LRWheeldis;
												break;
											case 6:
												item>>par.FBWheeldis;
												break;
											case 7:
												item>>par.LWheeldis;
												break;
											case 8:
												item>>par.Wheel_r;
												break;
											case 9:
												item>>par.Reduction_Ratio;
												break;
											case 10:
												item>>par.speedup;
												break;
											case 11:
												item>>par.speed;
												break;
											case 12:
												item>>par.JOYSTICK_SCALAR;
												break;
											case 13:
												item>>par.OBS_Front;
												break;
											case 14:
												item>>par.OBS_Side;
												break;
											case 15:
												item>>par.OBS_Front_limit;
												break;
											case 16:
												item>>par.OBS_Stop_Sec;
												break;
											case 17:
												item>>par.OBS_Wait_Sec;
												break;
											case 18:
												item>>par.OBS_Theta_Sec;
												break;
											case 19:
												item>>par.traffic_dis;
												break;
											case 20:
												item>>par.reMission_time;
												break;
											default:
												break;
										}

										//std::cout<<" recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;

										cnt_protect = count1;
										count1++;

									}

									//===========檢查封尾=======
									if(recv_pkg_subitem[0]=="E"&&count==end)
									{
										std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
										std::cout<<"par.CarName  "<< par.CarName_<<std::endl;
										std::cout<<"par.Carnumber  "<< par.Carnumber<<std::endl;
										std::cout<<"par.CarKind  "<< par.CarKind<<std::endl;
										std::cout<<"par.CarLength  "<< par.CarLength<<std::endl;
										if(cnt_protect != 20)
										{
											pkg_wrong=true;
											SetParameter=false;
											memset(&par,0,sizeof(par));
											break;
										}
									}
									else if(recv_pkg_subitem[0]!="E"&&count==end)
									{
										pkg_wrong=true;
										SetParameter=false;
										memset(&par,0,sizeof(par));
										break;
									}
								}

								//==============如果是Localparameter封包=============
								if(SetLoaclParameter)
								{
									std::cout<<"count  "<< count<<std::endl;
									float kp;
									if(count>0&& count<end)
									{
										std::istringstream item(recv_pkg_subitem[count1]);
										switch (count1){
											case 0:
												item>>kp;
												Lc_par.tracking_kp = kp;
												//item>>par.CarName_; //不要問為什麼 我也不知道為什麼會出錯
												break;
											case 1:
												item>>Lc_par.tracking_kd;
												break;
											case 2:
												item>>s_Car_packMode;
												if(s_Car_packMode == "ChinaMotor")
											Lc_par.Car_packMode = 0;
										else if(s_Car_packMode =="IVAM_Car_vlvr")
											Lc_par.Car_packMode = 0;
										else if(s_Car_packMode =="IVAM_Car_vw")
											Lc_par.Car_packMode = 1;
										else if(s_Car_packMode =="Boling_smallgray")
											Lc_par.Car_packMode = 2;
												break;
											case 3:
												item>>Lc_par.ChangeMode_dis_error;
												break;
											case 4:
												item>>Lc_par.ChangeMode_angular_error;
												break;
											case 5:
												item>>Lc_par.traffic_dis_error;
												break;
											case 6:
												item>>Lc_par.othermode_dis_error;
												break;
											case 7:
												item>>Lc_par.othermode_angular_error;
												break;
											case 8:
												item>>Lc_par.target_index_V_ratio;
												break;
											case 9:
												item>>Lc_par.target_index_distance;
												break;
											case 10:
												item>>Lc_par.network_card;
												break;
											case 11:
												item>>Lc_par.map_name;
												break;
											default:
												break;
										}

										//std::cout<<" recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;

										cnt_protect = count1;
										count1++;

									}

									//===========檢查封尾=======
									if(recv_pkg_subitem[0]=="E"&&count==end)
									{
										std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
										if(cnt_protect != 10)
										{
											pkg_wrong=true;
											SetLoaclParameter=false;
											memset(&Lc_par,0,sizeof(Lc_par));
											break;
										}
									}
									else if(recv_pkg_subitem[0]!="E"&&count==end)
									{
										pkg_wrong=true;
										SetLoaclParameter=false;
										memset(&Lc_par,0,sizeof(Lc_par));
										break;
									}
								}

								//============建立讀取地圖=============
								if(SetMap)
								{

									if(count>0&& count<end)
									{
										std::istringstream item(recv_pkg_subitem[count1]);
										switch (count1)
										{
												case 0:
													item>>commmand_buf;
													break;
												case 1:
													item>>Name_buf;
													break;
												default:
													break;
										}
										count1++;
									}


									//===========檢查封尾=======
									if(recv_pkg_subitem[0]=="E"&&count==end)
									{
										std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
									}
									else if(recv_pkg_subitem[0]!="E"&&count==end)
									{
										pkg_wrong=true;
										SetMap=false;
										commmand_buf = -1;
										break;
									}
								}

								//==============建立讀取地圖=============


								//============連線測試=============
								if(ConnectTest)
								{

									if(count>0&& count<end)
									{
										std::istringstream item(recv_pkg_subitem[count1]);

										item>>Connect_buf;
										count1++;
									}


									//===========檢查封尾=======
									if(recv_pkg_subitem[0]=="E"&&count==end)
									{
										std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
									}
									else if(recv_pkg_subitem[0]!="E"&&count==end)
									{
										pkg_wrong=true;
										ConnectTest=false;
										Connect_buf = "";
										break;
									}
								}

								//==============連線測試=============


								//==============交管重派任務=============
								if(RpMisson_pixel || RrMisson_real)
								{

									if(count>0&& count<end)
									{
										std::istringstream item(recv_pkg_subitem[count1]);
										switch (count1){
											case 0:
												item>>Start_ID;
												break;
											case 1:
												item>>End_ID;
												break;
											case 2:
												item>>End_x;
												break;
											case 3:
												item>>End_y;
												break;
											case 4:
												item>>End_z;
												break;

											default:
												break;
										}
										//std::cout<<" recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;

										count1++;
									}


									//===========檢查封尾=======
									if(recv_pkg_subitem[0]=="E"&&count==end)
									{
										std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
									}
									else if(recv_pkg_subitem[0]!="E"&&count==end)
									{
										pkg_wrong = true;
										RpMisson_pixel = false;
										RrMisson_real = false;
										Start_ID = -1;
										End_ID = -1;
										End_x = 0.0;
										End_y = 0.0;
										End_z = 0.0;
										break;
									}
								}

								//==============交管重派任務=============

								//============確認樓層=============
								if(Confirm_floor)
								{
										if(count>0&& count<end)
										{
											std::istringstream item(recv_pkg_subitem[count1]);

											item>>Floor_buf;
											count1++;
										}


										//===========檢查封尾=======
										if(recv_pkg_subitem[0]=="E"&&count==end)
										{
											std::cout<<"end recv_pkg_subitem= "<<count1 << " "<< recv_pkg_subitem[count1]<<std::endl;
										}
										else if(recv_pkg_subitem[0]!="E"&&count==end)
										{
											pkg_wrong=true;
											Confirm_floor=false;
											Floor_buf = 0;
											break;
										}
								}

								//==============確認樓層=============

							}
							static int wrong_cnt = 0;
							if(pkg_wrong){
								wrong_cnt ++;
								if(wrong_cnt == 10)
									ErrorState(11);
								break;
							}
							else
								wrong_cnt = 0;

							if(count>0 && count<end && (isMisson_pixel ||isMisson_real)) NodeSet.push_back(node_buf);


							count1=0;
							count++;

						}

						// std::cout<<" traffic_recv_buf.id=  "<<traffic_recv_buf.id << std::endl;
						// std::cout<<" traffic_recv_buf.GO=  "<<traffic_recv_buf.GO << std::endl;



						// std::cout<<" NodeSet.size=  "<<NodeSet.size() << std::endl;
						// std::cout<<" id.size=  "<<NodeSet[0].id << std::endl;
						// std::cout<<" type.size=  "<<NodeSet[0].type << std::endl;
						// std::cout<<" time.size=  "<<NodeSet[0].time << std::endl;
						// std::cout<<" id.size=  "<<NodeSet[1].id << std::endl;
						// std::cout<<" type.size=  "<<NodeSet[1].type << std::endl;
						// std::cout<<" id.size=  "<<NodeSet[2].id << std::endl;
						// std::cout<<" type.size=  "<<NodeSet[2].type << std::endl;
						// std::cout<<" btn_finish.size=  "<<NodeSet[2].btn_finish << std::endl;

						//===========PIC點轉換成世界座標點===============
						ros::Rate delay(100);
						if(isMisson_pixel)
						{
							for(int i=0;i<NodeSet.size();i++)
							{
								std::cout<<"---------------" << std::endl;


								float pixel_x=0;
								float pixel_y=0;
								float heading=0;

								pixel_x=NodeSet[i].id_pixel_x;
								pixel_y=NodeSet[i].id_pixel_y;
								heading=NodeSet[i].id_heading; // 0~3600  度（要除10才代表度）

								if(heading>1800)
									heading=heading-3600;

								heading=heading*M_PI/1800;



								recv_map<<pixel_x,pixel_y,heading;

								NodeSet[i].node_pose=getWorldCoordsPose(recv_map);


								//=======把資料傳去move_robot
								AnhungControl::Node_recv msg;

								msg.check=i+1;
								msg.value =NodeSet.size();
								msg.kin = NodeSet[i].kin;
								msg.id =NodeSet[i].id;
								msg.type =NodeSet[i].type;
								msg.time =NodeSet[i].time;
								msg.btn_finish =NodeSet[i].btn_finish;
								msg.x=NodeSet[i].node_pose.x();
								msg.y=NodeSet[i].node_pose.y();
								msg.z=NodeSet[i].node_pose.z();
								msg.line=NodeSet[i].line;
								msg.radius=NodeSet[i].radius;
								msg.map=NodeSet[i].map;
								msg.floor=NodeSet[i].floor;

								IdType_Publisher_.publish(msg);
								delay.sleep();


							}
						}

						if(isMisson_real)
						{
							for(int i=0;i<NodeSet.size();i++)
							{
								std::cout<<"---------------" << std::endl;


								float real_x=0;
								float real_y=0;
								float heading=0; //弳

								real_x=NodeSet[i].id_pixel_x;
								real_y=NodeSet[i].id_pixel_y;
								heading=NodeSet[i].id_heading;

								NodeSet[i].node_pose <<real_x,real_y,heading;


								//=======把資料傳去move_robot
								AnhungControl::Node_recv msg;

								msg.check=i+1;
								msg.value =NodeSet.size();
								msg.kin = NodeSet[i].kin;
								msg.id =NodeSet[i].id;
								msg.type =NodeSet[i].type;
								msg.time =NodeSet[i].time;
								msg.btn_finish =NodeSet[i].btn_finish;
								msg.x=NodeSet[i].node_pose.x();
								msg.y=NodeSet[i].node_pose.y();
								msg.z=NodeSet[i].node_pose.z();
								msg.line=NodeSet[i].line;
								msg.radius=NodeSet[i].radius;
								msg.map=NodeSet[i].map;
								msg.floor=NodeSet[i].floor;

								IdType_Publisher_.publish(msg);
								delay.sleep();


							}
						}


						if(isTraffic)
						{
							delay.sleep();
							std::cout<<"---------------" << std::endl;
							AnhungControl::traffic_recv msg;


							msg.id = traffic_recv_buf.id;
							msg.GO = traffic_recv_buf.GO;
							msg.speed = traffic_recv_buf.speed;

							trafficGO_Publisher_.publish(msg);
							delay.sleep();
							traffic_recv_buf.id = 0;
							traffic_recv_buf.GO = 0;
							traffic_recv_buf.speed = 0;
						}


						if(isClear_ObsMode_Misson)
						{
							std_msgs::Int8 msg;

							msg.data = clear_buf;


							std::cout<<" clear_buf=  "<<clear_buf << std::endl;

							//1 :清除任務
							//2 :關閉障，不會有障礙物減速問題
							//3 :極小避障距離（遇當障礙只會停下來）
							//4 :正常避障距離


							Clear_ObsMode_Publisher_.publish(msg);
							delay.sleep();
						}

						if(SetParameter)
						{
							std::fstream fout;
							char *file = const_cast<char *>(CarParameterPATH.c_str());
							fout.open(file,std::fstream::out);

							fout<<"CarName"<<std::endl;
							fout<<par.CarName_<<std::endl;
							fout<<"Carnumber"<<std::endl;
							fout<<par.Carnumber<<std::endl;
							fout<<"CarKind"<<std::endl;
							fout<<par.CarKind<<std::endl;
							fout<<"CarLength"<<std::endl;
							fout<<par.CarLength<<std::endl;
							fout<<"CarWidth"<<std::endl;
							fout<<par.CarWidth<<std::endl;
							fout<<"LRWheeldis"<<std::endl;
							fout<<par.LRWheeldis<<std::endl;
							fout<<"FBWheeldis"<<std::endl;
							fout<<par.FBWheeldis<<std::endl;
							fout<<"LWheeldis"<<std::endl;
							fout<<par.LWheeldis<<std::endl;
							fout<<"Wheel_r"<<std::endl;
							fout<<par.Wheel_r<<std::endl;
							fout<<"Reduction_Ratio"<<std::endl;
							fout<<par.Reduction_Ratio<<std::endl;
							fout<<"speedup"<<std::endl;
							fout<<par.speedup<<std::endl;
							fout<<"speed"<<std::endl;
							fout<<par.speed<<std::endl;
							fout<<"JOYSTICK_SCALAR"<<std::endl;
							fout<<par.JOYSTICK_SCALAR<<std::endl;
							fout<<"OBS_Front"<<std::endl;
							fout<<par.OBS_Front<<std::endl;
							fout<<"OBS_Side"<<std::endl;
							fout<<par.OBS_Side<<std::endl;
							fout<<"OBS_Front_limit"<<std::endl;
							fout<<par.OBS_Front_limit<<std::endl;
							fout<<"OBS_Stop_Sec"<<std::endl;
							fout<<par.OBS_Stop_Sec<<std::endl;
							fout<<"OBS_Wait_Sec"<<std::endl;
							fout<<par.OBS_Wait_Sec<<std::endl;
							fout<<"OBS_Theta_Sec"<<std::endl;
							fout<<par.OBS_Theta_Sec<<std::endl;
							fout<<"traffic_dis"<<std::endl;
							fout<<par.traffic_dis<<std::endl;
							fout<<"reMission_time"<<std::endl;
							fout<<par.reMission_time<<std::endl;

							fout.close();

							//ErrorState(1);
							LoadCarParameter(CarParameterPATH);

							std_msgs::Int8 msg;
							msg.data = ReloadCarParameter;
							Trigger_Publisher_.publish(msg);
							delay.sleep();
							Opensend();






						}
						if(SetLoaclParameter)
						{
							std::fstream fout;
							char *file = const_cast<char *>(LocalparPATH.c_str());
							fout.open(file,std::fstream::out);

							fout<<"kp"<<std::endl;
							fout<<Lc_par.tracking_kp<<std::endl;
							fout<<"kd"<<std::endl;
							fout<<Lc_par.tracking_kd<<std::endl;
							fout<<"CAR_"<<std::endl;
							fout<<s_Car_packMode<<std::endl;
							fout<<"ChangeMode_dis_error"<<std::endl;
							fout<<Lc_par.ChangeMode_dis_error<<std::endl;
							fout<<"ChangeMode_angular_error"<<std::endl;
							fout<<Lc_par.ChangeMode_angular_error<<std::endl;
							fout<<"traffic_dis_error"<<std::endl;
							fout<<Lc_par.traffic_dis_error<<std::endl;
							fout<<"othermode_dis_error"<<std::endl;
							fout<<Lc_par.othermode_dis_error<<std::endl;
							fout<<"othermode_angular_error"<<std::endl;
							fout<<Lc_par.othermode_angular_error<<std::endl;
							fout<<"target_index_V_ratio"<<std::endl;
							fout<<Lc_par.target_index_V_ratio<<std::endl;
							fout<<"target_index_distance"<<std::endl;
							fout<<Lc_par.target_index_distance<<std::endl;
							fout<<"network_card"<<std::endl;
							fout<<Lc_par.network_card<<std::endl;
							fout<<"map_name"<<std::endl;
							fout<<Lc_par.map_name<<std::endl;

							fout.close();

							//ErrorState(1);
							SetLocalpar(LocalparPATH);

							std_msgs::Int8 msg;
							msg.data = ReloadLocalParameter;
							Trigger_Publisher_.publish(msg);
							delay.sleep();

						}
						if(SetMap)
						{
							std::string command_str;
							switch (commmand_buf) {
								case 0:
									command_str ="Create Map";
									break;
								case 1:
									command_str ="Save Map";
									break;
								case 2:
									command_str ="Load Map";
									break;
								case 3:
									command_str ="navigation=true";
									break;
								case 4:
									command_str ="navigation=false";
									break;
								default:
									break;
							}
							//0 :Create Map
							//1 :Save Map
							//2 :Load Map
							//3 :navigation=true
							//4 :navigation=false



							std::cout<<" command_str "<<command_str<<std::endl;
							std::cout<<" Name_buf "<<Name_buf<<std::endl;

							AnhungControl::setmap_ctr msg;
							msg.type = command_str;
							msg.Name = Name_buf;
							Command_Publisher_.publish(msg);

							if(command_str == "Load Map")
							{
								Mapname = Name_buf + "_0";
							}

						}

						if(Confirm_floor)
						{
								std::cout<<" Floor_buf "<<Floor_buf<<std::endl;
								std_msgs::Int8 msg;
								msg.data = Floor_buf;
								floor_Publisher_.publish(msg);
						}

						if(ConnectTest)
						{
							std::string head ="I;";
							std::string tail =";E";
							if(Connect_buf == "check")
							{
								//std::string IP = "XXX.XXX.XXX.XXX";
								std::string registration_buf =int2str(registration);
								std::string DataFlow = head + CarName + "," + Carnumber + "," + IP_buf + "," + "connect" + "," + registration_buf + tail ;

								std::cout<<DataFlow<<std::endl;


								char *all_data = new char[DataFlow.size() + 1];
								std::strcpy(all_data, DataFlow.c_str());
								sendto(sfd_, all_data, std::strlen(all_data), 0, (struct sockaddr *)&si_other_, slen_);
							}


						}

						if(RpMisson_pixel)
						{
							NODE_recv node_buf;
							node_buf.id = Start_ID;
							node_buf.type = 0;
							node_buf.id_pixel_x = robot_pose_.x();
							node_buf.id_pixel_y = robot_pose_.y();
							node_buf.id_heading = robot_pose_.z();
							node_buf.kin = "omni";
							node_buf.line = 0;
							node_buf.radius = 100.0;


							NodeSet.push_back(node_buf);
							memset(&node_buf,0,sizeof(node_buf));


							float heading=0;

							heading=End_z; // 0~3600  度（要除10才代表度）

							if(heading>1800)
								heading=heading-3600;

							heading=heading*M_PI/1800;

							recv_map<<End_x,End_y,heading;

							Eigen::Vector3f real_pose = getWorldCoordsPose(recv_map);

							node_buf.id = End_ID;
							node_buf.type = 8;
							node_buf.id_pixel_x = real_pose.x();
							node_buf.id_pixel_y = real_pose.y();
							node_buf.id_heading = real_pose.z();
							node_buf.kin = "omni";
							node_buf.line = 0;
							node_buf.radius = 100.0;

							NodeSet.push_back(node_buf);
							memset(&node_buf,0,sizeof(node_buf));





							for(int i=0;i<NodeSet.size();i++)
							{
								std::cout<<"---------------" << std::endl;


								//=======把資料傳去move_robot
								AnhungControl::Node_recv msg;

								msg.check=i+1;
								msg.value =NodeSet.size();
								msg.kin = NodeSet[i].kin;
								msg.id =NodeSet[i].id;
								msg.type =NodeSet[i].type;
								msg.time =NodeSet[i].time;
								msg.btn_finish =NodeSet[i].btn_finish;
								msg.x=NodeSet[i].node_pose.x();
								msg.y=NodeSet[i].node_pose.y();
								msg.z=NodeSet[i].node_pose.z();
								msg.line=NodeSet[i].line;
								msg.radius=NodeSet[i].radius;

								IdType_Publisher_.publish(msg);
								delay.sleep();


							}
						}

						if(RrMisson_real)
						{
							NODE_recv node_buf;
							node_buf.id = Start_ID;
							node_buf.type = 0;
							node_buf.id_pixel_x = robot_pose_.x();
							node_buf.id_pixel_y = robot_pose_.y();
							node_buf.id_heading = robot_pose_.z();
							node_buf.kin = "omni";
							node_buf.line = 0;
							node_buf.radius = 100.0;


							NodeSet.push_back(node_buf);
							memset(&node_buf,0,sizeof(node_buf));


							node_buf.id = End_ID;
							node_buf.type = 8;
							node_buf.id_pixel_x = End_x;
							node_buf.id_pixel_y = End_y;
							node_buf.id_heading = End_z;
							node_buf.kin = "omni";
							node_buf.line = 0;
							node_buf.radius = 100.0;

							NodeSet.push_back(node_buf);
							memset(&node_buf,0,sizeof(node_buf));





							for(int i=0;i<NodeSet.size();i++)
							{
								std::cout<<"---------------" << std::endl;


								//=======把資料傳去move_robot
								AnhungControl::Node_recv msg;

								msg.check=i+1;
								msg.value =NodeSet.size();
								msg.kin = NodeSet[i].kin;
								msg.id =NodeSet[i].id;
								msg.type =NodeSet[i].type;
								msg.time =NodeSet[i].time;
								msg.btn_finish =NodeSet[i].btn_finish;
								msg.x=NodeSet[i].node_pose.x();
								msg.y=NodeSet[i].node_pose.y();
								msg.z=NodeSet[i].node_pose.z();
								msg.line=NodeSet[i].line;
								msg.radius=NodeSet[i].radius;

								IdType_Publisher_.publish(msg);
								delay.sleep();

							}
						}




						//事情做完，清掉
						pkg_wrong = false;
						count1=0;
						count=0;
						NodeSet.clear();
						clear_buf=0;
						isMisson_pixel = false;
						isMisson_real = false;
						isTraffic=false;
						isClear_ObsMode_Misson=false;
						Confirm_floor = false;
						SetParameter=false;
						SetLoaclParameter = false;
						SetMap=false;
						ConnectTest = false;
						RpMisson_pixel = false;
						RrMisson_real = false;
				}
			}

			r.sleep();
		}
	}


}
void Control::ErrorState(int i)
{
	// 0 :無車子參數
	// 1 :車子參數建立成功
	// 2 :車子參數讀取失敗
	// 3 :serial port開啟錯誤
	// 4 :tcgetattr 序列埠參數讀取錯誤
	// 5 :請再傳一次
	// 6 :雷達封包MISS
	// 7 :迷航
	// 8 :封包衝突
	// 9 :DeadLock 警告
	//10 :線模式有誤
	//11 :封包ERROR 請檢查封包

	//    switch (i) {
	//    case 0:

	//        break;
	//    case 1:

	//        break;
	//    default:
	//        break;
	//    }

	std::string head ="E;";
	std::string tail =";E";



	std::string i_buf =int2str(i);

	// if(i ==0)
	//     CarName = "X";

	std::string DataFlow = head + CarName + ","+ IP_buf + "," + i_buf + tail ;

	std::cout<<DataFlow<<std::endl;


	char *all_data = new char[DataFlow.size() + 1];
	std::strcpy(all_data, DataFlow.c_str());
	sendto(sfd_, all_data, std::strlen(all_data), 0, (struct sockaddr *)&si_other_, slen_);

}

void Control::ReMissionStateCallback(const std_msgs::Int8& msg)
{
	int ReMissionState = msg.data;
	std::string State;

	switch(ReMissionState)
	{
		case 0:
			State ="obs";
			break;
		case 1:
			State ="traffic";
			break;
		case 2:
			State ="done";
			break;
	}

	std::string head ="R;";
	std::string tail =";E";


	std::string DataFlow = head + CarName + "," + Carnumber + "," + State + tail ;

	std::cout<<DataFlow<<std::endl;


	char *all_data = new char[DataFlow.size() + 1];
	std::strcpy(all_data, DataFlow.c_str());
	sendto(sfd_, all_data, std::strlen(all_data), 0, (struct sockaddr *)&si_other_, slen_);




}
void Control::ErrorCallback(const std_msgs::Int8& msg)
{
	ErrorState( msg.data );

}

void Control::mapCallback(const nav_msgs::OccupancyGrid& map)
{
	//std::cout<<" map map  kolk"<<std::endl;
	if(isLoadMap){

		if(isSetMapTrans == false){
			map_resloution  = map.info.resolution;
			map_width_size  = map.info.width;
			map_height_size = map.info.height;
			Eigen::Vector2f StartCoord(map_width_size*0.50*map_resloution , map_height_size*0.50*map_resloution);
			setMapTransformation(StartCoord, map_resloution);
			isSetMapTrans = true;
		}


		std::string path(argv);

		path = path + "/sendmap.pgm";

		FILE* out = fopen("/home/user/sendmap.pgm", "w");
		if (!out)
		{
			std::cout<<"Couldn't save map file"<<std::endl;
		}
		std::fprintf(out, "P5\n# CREATOR: AnhungControl.cpp %.3f m/pix\n%d %d\n255\n", map.info.resolution, map.info.width, map.info.height);
		for(int j=0; j<map.info.height; j++){
			for(int i=0; i<map.info.width; i++){

				int value = map.data[(map.info.height - j -1)*map.info.width + i];
				if(value == -1)
					std::fputc(205, out);
				else if(value > 50)
					std::fputc(0, out);
				else
					std::fputc(254, out);
			}
		}
		std::fclose(out);

		isLoadMap = false;

	}



}



void Control::slamoutCallback(const geometry_msgs::PoseStamped& msg)
{
	static int count =0;
	count+=1;

	tf::Pose pose_tf;
	tf::poseMsgToTF(msg.pose, pose_tf);
	robot_pose_ = Eigen::Vector3f(msg.pose.position.x, msg.pose.position.y, tf::getYaw(pose_tf.getRotation()));
	//ROS_INFO("Robot pose x: %f y: %f yaw: %f", robot_pose_[0], robot_pose_[1], robot_pose_[2]);

	//=============送封包============
	std::string head ="L;";
	std::string tail =";E";




	std::string real_pose_x = float2str(robot_pose_.x() );
	std::string real_pose_y = float2str(robot_pose_.y() );
	std::string real_pose_z = float2str(robot_pose_.z() );



	Eigen::Vector3f robot_pose_p = getMapCoordsPose(robot_pose_);

	robot_pose_p.z()=robot_pose_p.z()*1800/M_PI;

	//std::cout<<" robot_pose_p  "<< robot_pose_p<<std::endl;



	std::string robot_pose_x = int2str(int(robot_pose_p.x() ));
	std::string robot_pose_y = int2str(int(robot_pose_p.y() ));
	std::string robot_pose_z = int2str(int(robot_pose_p.z() ));



	std::string missState;



	if(isMissing==0)
		missState = "Normal";
	else
		missState = "Missing" ;


	std::string State;

	switch(Command_STATE)
	{
		case Command_STATE_IDLE:
			State ="IDLE";
			break;
		case Command_STATE_TRACKING:
			State ="TRACKING";
			break;
		case Command_STATE_Traffic:
			State ="Traffic";
			break;
		case Command_STATE_Loading:
			State ="Loading";
			break;
		case Command_STATE_unLoading:
			State ="unLoading";
			break;
		case Command_STATE_TemporaryMode:
			State ="TemporaryStop";
			break;
		case Command_STATE_FreeLoading:
			State ="FreeLoading";
			break;
	  case Command_STATE_virtual_Traffic:
			State ="VirtualTraffic";
			break;
	  case Command_Elevator_Entrance:
			State ="ElevatorEntrance";
			break;
		case Command_Elevator_Inside:
			State ="ElevatorInside";
			break;
		case Command_Samefloor_ChangeMap:
			State ="SamefloorChangeMap";
			break;
	}

	if(isReveice_joystick){

		switch(btn_id){
			case PUSE_BUTTON_A:
				State = "Joystick_diff";

				break;

			case PUSE_BUTTON_B:
				State = "Joystick_omni";
				break;

			case PUSE_BUTTON_X:
				State = "turn_left";
				break;

			case PUSE_BUTTON_Y:
				State = "turn_right";
				break;

			default:
				break;


		}

		isReveice_joystick = false;
	}

	//任務時間計算
	static bool time_cnt = false;


	switch(Command_STATE)
	{
		case Command_STATE_IDLE:
			//State ="IDLE";
			time_cnt = false;
			all_missiontime += missiontime;
			missiontime = 0;

			break;
		case Command_STATE_TRACKING:
			if(!time_cnt)
			{
				time_cnt = true;
				mission_begin_time = ros::Time::now ();

			}
			missiontime = (ros::Time::now () - mission_begin_time).toSec ();
			//State ="TRACKING";

			break;
	}


	std::string pos_id;
	std::string robot_V;

	pos_id=int2str(Command_id);

	robot_V =float2str(V_now);

	std::string Voltage_st = float2str(Voltage);
	std::string Current_st = float2str(Current);
	std::string RelativeSOC1_st = float2str(RelativeSOC1);
	std::string RelativeSOC2_st = float2str(RelativeSOC2);
	std::string RelativeSOC3_st = float2str(RelativeSOC3);
	std::string RelativeSOC4_st = float2str(RelativeSOC4);
	std::string AbsoluteSOC1_st = float2str(AbsoluteSOC1);
	std::string AbsoluteSOC2_st = float2str(AbsoluteSOC2);
	std::string AbsoluteSOC3_st = float2str(AbsoluteSOC3);
	std::string AbsoluteSOC4_st = float2str(AbsoluteSOC4);
	std::string Temp1_st = float2str(Temp1);
	std::string Temp2_st = float2str(Temp2);
	std::string Temp3_st = float2str(Temp3);
	std::string Temp4_st = float2str(Temp4);




	if(count==5)
	{
		//總開機時間計算
		int opentime = (ros::Time::now () - begin_time).toSec ();
		std::string Opentime =  int2str(opentime);
		//std::cout<<"Opentime  "<< Opentime<<std::endl;

		int now_opentime = last_all_sec + opentime;
		int now_missiontime = last_all_missiontime + all_missiontime + missiontime;

		std::fstream fout;
		char *file = const_cast<char *>(time_path.c_str());


		//總任務時間計算
		//int missiontime = (ros::Time::now () - mission_begin_time).toSec ();

		//每進一次刷新一次
		fout.open(file, std::fstream::out);
		fout<<Time_date<<std::endl;
		fout<<now_opentime<<std::endl;
		fout<<now_missiontime<<std::endl;
		now_opentime = now_opentime / 60;//轉成分鐘
		now_missiontime = now_missiontime / 60;//轉成分鐘
		std::string send_opentime = int2str(now_opentime);
		std::string send_missiontime = int2str(now_missiontime);



		// std::string DataFlow = head + CarName + "," + Carnumber + "," + IP_buf + "," + Mapname + ","
		// +robot_pose_x + "," + robot_pose_y + "," + robot_pose_z  + ","
		// + real_pose_x + "," + real_pose_y + "," + real_pose_z + ","
		// + robot_V + "," + missState + "," + State + "," + pos_id + ","
		// + Voltage_st + "," + Current_st + ","
		// + RelativeSOC1_st + "," + RelativeSOC2_st + "," + RelativeSOC3_st + "," + RelativeSOC4_st + ","
		// + AbsoluteSOC1_st + "," + AbsoluteSOC2_st + "," + AbsoluteSOC3_st + "," + AbsoluteSOC4_st + ","
		// + Temp1_st + "," + Temp2_st + "," + Temp3_st + "," + Temp4_st + ","
		// + send_opentime + "," + send_missiontime
		// + tail ;

		std::string DataFlow = head +  Mapname + ","
		+robot_pose_x + "," + robot_pose_y + "," + robot_pose_z  + ","
		+ robot_V + "," + missState + "," + State
		+ tail ;

		char *all_data = new char[DataFlow.size() + 1];
		std::strcpy(all_data, DataFlow.c_str());

		std::cout<<DataFlow<<std::endl;
		std::cout<<robot_pose_<<std::endl;


		sendto(sfd_, all_data, std::strlen(all_data), 0, (struct sockaddr *)&si_other_, slen_);

		count=0;

	}

	if(SendNode)
	{
		std::cout<<"SendNode2"<<std::endl;
		std::string DataFlow = "N;"  + Mapname + "," +robot_pose_x + "," + robot_pose_y + "," + robot_pose_z + "," + real_pose_x + "," + real_pose_y + "," + real_pose_z  + tail ;


		char *all_data = new char[DataFlow.size() + 1];
		std::strcpy(all_data, DataFlow.c_str());

		std::cout<<DataFlow<<std::endl;

		sendto(sfd_, all_data, std::strlen(all_data), 0, (struct sockaddr *)&si_other_, slen_);
		SendNode = false;

	}
}


std::string Control::int2str(int i)
{
	std::string s;
	std::stringstream ss(s);
	ss << i;

	return ss.str();
}

std::string Control::float2str(float i)
{
	std::string s;
	std::stringstream ss(s);
	ss << i;

	return ss.str();
}
void Control::STATECallback( const AnhungControl::state& msg )
{
	Command_STATE = msg.state;
	Command_id    = msg.id;

}


void Control::MissMsgCallback(const std_msgs::Int16& msg)
{
	static bool miss_cnt = false;
	isMissing = msg.data;
	//ROS_INFO("  isMissing   ");

	//std::cout<< "isMissing  "<<isMissing <<std::endl;

	if(isMissing && !miss_cnt)
	{
		miss_cnt = true;
		ErrorState(7);

	}
	else
		miss_cnt = false;



}

void Control::vCallback(const std_msgs::Float32& msg)
{

	V_now = msg.data;

}
void Control::joystickCallback(const AnhungControl::joystick& joystick)
{
	if(PUSE_BUTTON_RB != joystick.btn_id && PUSE_BUTTON_START!= joystick.btn_id)
		btn_id = joystick.btn_id;
	else
	{
		if(PUSE_BUTTON_START == joystick.btn_id)
			protect_SendNode = !protect_SendNode;

		if(protect_SendNode)
		{

			if(PUSE_BUTTON_RB == joystick.btn_id)
			{
				SendNode = true;
				std::cout<<"GO"<<std::endl;
			}
		}

	}

	isReveice_joystick = true;
}
void Control::BatteryCallback(const AnhungControl::Battery& msg)
{
	//move_robot::Battery msg;
	if(msg.Voltage >60 ||msg.Current >5 || msg.RelativeSOC1 >150 || msg.RelativeSOC2 >150 || msg.RelativeSOC3 >150 || msg.RelativeSOC4 >150 || msg.AbsoluteSOC1 >150 || msg.AbsoluteSOC2 >150|| msg.AbsoluteSOC3 >150|| msg.AbsoluteSOC4 >150)
	{
		Voltage = msg.Voltage ;
		Current = msg.Current ;
		RelativeSOC1 = msg.RelativeSOC1 ;
		RelativeSOC2 = msg.RelativeSOC2 ;
		RelativeSOC3 = msg.RelativeSOC3 ;
		RelativeSOC4 = msg.RelativeSOC4 ;
		AbsoluteSOC1 = msg.AbsoluteSOC1 ;
		AbsoluteSOC2 = msg.AbsoluteSOC2 ;
		AbsoluteSOC3 = msg.AbsoluteSOC3 ;
		AbsoluteSOC4 = msg.AbsoluteSOC4 ;
		Temp1 = msg.Temp1 ;
		Temp2 = msg.Temp2 ;
		Temp3 = msg.Temp3 ;
		Temp4 = msg.Temp4 ;

	}
	else
	{
		Voltage = 0;
		Current = 0;
		RelativeSOC1 = 0;
		RelativeSOC2 = 0;
		RelativeSOC3 = 0;
		RelativeSOC4 = 0;
		AbsoluteSOC1 = 0;
		AbsoluteSOC2 = 0;
		AbsoluteSOC3 = 0;
		AbsoluteSOC4 = 0;
		Temp1 = 0;
		Temp2 = 0;
		Temp3 = 0;
		Temp4 = 0;
	}

}
void Control::MapnameCallback(const std_msgs::String& msg)
{
	Mapname = msg.data;
}

int Control::recvtimeout(int ss, char *buf, int len, int timeout)
{
	fd_set fds;
	int n;
	struct timeval tv;

	FD_ZERO(&fds);
	FD_SET(ss, &fds);

	tv.tv_sec = 0;
	tv.tv_usec = timeout;

	n = select(ss+1, &fds, NULL, NULL, &tv);
	if(n == 0) return -2; //timeout;
	if(n == -1) return -1; //error

	return recv(ss, buf, len, 0);
}

void Control::setMapTransformation(const Eigen::Vector2f &StartCoord, float cellLength)
{
	float mid_offset_x = StartCoord[0];
	float mid_offset_y = StartCoord[1];
	float scaleToMap = 1.0f / cellLength;
	worldTmap = Eigen::AlignedScaling2f(scaleToMap, scaleToMap) * Eigen::Translation2f(mid_offset_x, mid_offset_y);
	mapTworld = worldTmap.inverse();
}

Eigen::Vector2f Control::getWorldCoords(const Eigen::Vector2f& mapCoords)
{
	return mapTworld * mapCoords;
}

Eigen::Vector2f Control::getMapCoords(const Eigen::Vector2f& worldCoords)
{
	return worldTmap * worldCoords;
}

Eigen::Vector3f Control::getWorldCoordsPose(const Eigen::Vector3f& mapPose)
{

	Eigen::Vector2f worldCoords (mapTworld * mapPose.head<2>());
	return Eigen::Vector3f(worldCoords[0], -1*worldCoords[1], mapPose[2]);
}

Eigen::Vector3f Control::getMapCoordsPose(const Eigen::Vector3f& worldPose)
{
	Eigen::Vector2f mapCoords (worldTmap * worldPose.head<2>());
	//std::cout<<"pixel  "<<Eigen::Vector3f(mapCoords[0], max_map_height - mapCoords[1], worldPose[2])<<std::endl;
	return Eigen::Vector3f(mapCoords[0], max_map_height - mapCoords[1], worldPose[2]);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Anhung_Contorl_Center");
	ros::Time::init();

	ros::NodeHandle n;

	Control *myControl;

	if(argc < 3){

		ROS_INFO("usage: [<IP address>] [<Port>]");

		return 0;

	}
	else{
		int port = std::atoi(argv[2]);
		myControl = new Control(argv[0], argv[1], port);
		// if(myControl->isSocketOpen_ == false){
		// 	ROS_INFO("Socket failed");
		// 	delete myControl;
		// 	return 0;
		// }
	}


	ros::spin();

	delete myControl;


	return 0;

}
void Control::setTime()
{
	time_t t = time(0);
	char tmp[64];
	strftime( tmp, sizeof(tmp), "%Y/%m/%d %X %A 本年第%j天 %z",localtime(&t) );
	std::string NowTime(tmp);
	std::cout<<"NowTime = "<<NowTime<<std::endl;


	std::string Time_cut;
	std::string recv_time[5];
	std::stringstream cut(tmp);
	int count = 0;
	while(getline(cut,Time_cut,' '))
	{
			recv_time[count]=Time_cut;
			count++;
	}
	Time_date = recv_time[0];
	Time_ = recv_time[1];

	std::cout<<" Time_date= "<<Time_date <<std::endl;
	std::cout<<" Time_= "<<Time_ <<std::endl;

	time_path = TitlePath + "/time";


	std::string s_date;
	std::string s_last_all_sec;
	std::string s_last_mission_all_sec;


	std::fstream fout;
	char *file = const_cast<char *>(time_path.c_str());
	fout.open(file, std::fstream::in);

	if(!fout.is_open())
	{
		fout.open(file, std::fstream::out);
		fout<<Time_date<<std::endl;
		fout<<"0"<<std::endl;
		fout<<"0"<<std::endl;

		last_all_sec = 0;
		last_all_missiontime = 0;
	}
	else{
		std::getline(fout, s_date);
		std::getline(fout, s_last_all_sec);
		std::getline(fout, s_last_mission_all_sec);
		if(s_date == Time_date)
		{
			last_all_sec = std::atoi(s_last_all_sec.c_str());
			last_all_missiontime = std::atoi(s_last_mission_all_sec.c_str());
		}
		else
		{
			fout.close();
			fout.open(file, std::fstream::out);
			//fout.seekg(10,std::ios::beg);
			fout<<Time_date<<std::endl;
			fout<<"0"<<std::endl;
			fout<<"0"<<std::endl;
			last_all_sec = 0;
			last_all_missiontime = 0;
		}

		ROS_INFO("YES");

	}

	std::cout<<"last_all_sec "<<last_all_sec<<std::endl;
	std::cout<<"last_all_missiontime "<<last_all_missiontime<<std::endl;
	fout.close();

}
void Control::LoadTitlePath()
{
	std::string NOWPath = ros::package::getPath("AnhungControl");

	std::string PATH_par;
	std::string recv_pkg[100];

	int count=0;
	std::stringstream cut(NOWPath);
	while(getline(cut,PATH_par,'/'))
	{
		recv_pkg[count]=PATH_par;
		count++;
	}

	TitlePath = "/" + recv_pkg[1] + "/" + recv_pkg[2] + "/" + recv_pkg[3];
	std::cout<<"TitlePath  " <<TitlePath <<std::endl;

}
void Control::Opensend()
{
	std::string head ="I;";
	std::string tail =";E";

	std::string IP = IP_buf;
	std::string registration_buf =int2str(registration);
	std::string DataFlow = head + CarName + "," + Carnumber + "," + IP + "," + "open" + "," + registration_buf + tail ;

	std::cout<<DataFlow<<std::endl;


	char *all_data = new char[DataFlow.size() + 1];
	std::strcpy(all_data, DataFlow.c_str());
	sendto(sfd_, all_data, std::strlen(all_data), 0, (struct sockaddr *)&si_other_, slen_);
}
void Control::SearchIP()
{
	int sockfd;
	struct sockaddr_in *addr;
	struct ifreq if_info;
	char buff[INET_ADDRSTRLEN];

	sockfd =socket(AF_INET, SOCK_DGRAM, 0);
        strcpy(if_info.ifr_name, Lc_par.network_card.c_str());
	ioctl(sockfd, SIOCGIFADDR, &if_info);

	addr = (struct sockaddr_in *)&if_info.ifr_addr;
	inet_ntop(AF_INET,&addr->sin_addr,buff,sizeof(buff)) ;

	IP_buf = buff;

}
void Control::sendScale()
{
	std::string head ="Sc;";
	std::string tail =";E";

	std::string IP = IP_buf;

	Eigen::Vector3f zero(0.0, 0.0, 0.0);
	Eigen::Vector3f unit_real_dis(1.0, 0.0, 0.0);
	Eigen::Vector3f unit_pixel_dis = getMapCoordsPose(unit_real_dis) - getMapCoordsPose(zero);


	std::string scale = float2str(unit_pixel_dis.x());

	std::string DataFlow = head + CarName + "," + Carnumber + "," + IP + "," + Mapname + "," + scale +  tail ;

	std::cout<<DataFlow<<std::endl;


	char *all_data = new char[DataFlow.size() + 1];
	std::strcpy(all_data, DataFlow.c_str());
	sendto(sfd_, all_data, std::strlen(all_data), 0, (struct sockaddr *)&si_other_, slen_);


}
