


class diff_vw: public Move_Robot
{
	public:
		diff_vw(char *dev_name, int Baudrate);
		~diff_vw();

		ros::Subscriber Clear_ObsModeSubscriber_;

		ros::Publisher cmd_vel_pub; // kevin

		void RevProcess(double receive_period);
		//Callback
		void timerCallback(const ros::TimerEvent& event);
		void ClearCallback(const std_msgs::Int8& msg);
		void laserCallback(const sensor_msgs::LaserScan& scan);
		void joystickCallback(const move_robot::joystick& joystick);
		void TriggerCallback_(const std_msgs::Int8& msg);


		//state
		void State_Machine(int& state);
		void Idle();
		void Stop();

		//tracking
		bool Navigation_move(std::vector<SUB_MISSONPATH_SUBPOINT> & myAllSubPath, bool isReSet);
		bool Tracking_Angle_Init(int &subpath_index, bool isReSet);
		bool Tracking_Trajectory(int &subpath_index, bool isReSet);
		bool Trajectory_Tracking(int &subpath_index, bool isReSet);

		//OBS
		void ObsDetect(std::vector<Eigen::Vector3f> &i_obs_container,
				std::vector<Eigen::Vector3f> &i_obs_region_0,
				std::vector<Eigen::Vector3f> &i_obs_region_1,
				std::vector<Eigen::Vector3f> &i_obs_region_2,
				std::vector<Eigen::Vector3f> &i_obs_region_3,
				std::vector<Eigen::Vector3f> &i_obs_region_4,
				std::vector<Eigen::Vector3f> &i_obs_gap_0,
				std::vector<Eigen::Vector3f> &i_obs_gap_1,
				std::vector<Eigen::Vector3f> &i_obs_gap_2,
				std::vector<Eigen::Vector3f> &i_obs_gap_3,
				std::vector<Eigen::Vector3f> &i_obs_gap_4,
				std::vector<Eigen::Vector3f> &all_point);


		void ObsStateMachine(std::vector<Eigen::Vector3f> &i_obs_region_0,
				std::vector<Eigen::Vector3f> &i_obs_region_1,
				std::vector<Eigen::Vector3f> &i_obs_region_2,
				std::vector<Eigen::Vector3f> &i_obs_region_3,
				std::vector<Eigen::Vector3f> &i_obs_region_4,
				std::vector<Eigen::Vector3f> &i_obs_gap_0,
				std::vector<Eigen::Vector3f> &i_obs_gap_1,
				std::vector<Eigen::Vector3f> &i_obs_gap_2,
				std::vector<Eigen::Vector3f> &i_obs_gap_3,
				std::vector<Eigen::Vector3f> &i_obs_gap_4,
				std::vector<Eigen::Vector3f> &all_point);


		void RePlanning(int &subpath_index,
				std::vector<Eigen::Vector3f> &i_obs_region_0,
				std::vector<Eigen::Vector3f> &i_obs_region_1,
				std::vector<Eigen::Vector3f> &i_obs_region_2,
				std::vector<Eigen::Vector3f> &i_obs_region_3,
				std::vector<Eigen::Vector3f> &i_obs_region_4,
				std::vector<Eigen::Vector3f> &all_point);

		bool FindKeyPoint(Eigen::Vector3f& target, Eigen::Vector3f& key_p,
				std::vector<Eigen::Vector3f> &i_obs_region_0,
				std::vector<Eigen::Vector3f> &i_obs_region_2,
				std::vector<Eigen::Vector3f> &i_obs_region_4,
				std::vector<Eigen::Vector3f> &all_point);

		void CreateNewPath(Eigen::Vector3f& target, Eigen::Vector3f& key_point, int subpath_index, std::vector<Eigen::Vector3f> &new_path);
		bool WaitObsLeave();
		bool WaitDeadlock();
		bool AvoidObs();

		//joystick
		void joystick_move();

		void Calculate_odom();




	private:

	float vl, vr ;
	float global_v ;

	float Rev_V, Rev_W, Rev_a, Rev_th;
	int type;

	float stop_w_max;
	float stop_w_min;
	float move_w_max;
	float predict_turn_dis;
	float predict_turn_R;




};

diff_vw::diff_vw(char* dev_name, int Baudrate):Move_Robot(dev_name, Baudrate)
{
	vl = 0.0;
    vr = 0.0;
	global_v = 0.0;

	Rev_V = 0.0;
	Rev_W = 0.0;
	Rev_a = 0.0;
	Rev_th =0.0;
	type = 0;

	stop_w_max = 0.65;
	stop_w_min = 0.05;
	move_w_max = 0.8;
	predict_turn_dis = 1.5;
	predict_turn_R = 1.0;



	std::cout<<"diff_vw"<<std::endl;
	Clear_ObsModeSubscriber_=node_.subscribe("Clear_ObsMode", 10, &diff_vw::ClearCallback,this);
	cmd_vel_pub = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1000); // kevin
	laserSubscriber_ = node_.subscribe("scan", 10, &diff_vw::laserCallback, this);
	JoysickSubscriber_ = node_.subscribe("joystick", 5, &diff_vw::joystickCallback, this);

	receive_thread_ = new boost::thread(boost::bind(&diff_vw::RevProcess, this, 0.01));
	control_timer = node_.createTimer(ros::Duration(time_sample), &diff_vw::timerCallback, this);
	TriggerSubscriber_=node_.subscribe("Trigger", 5, &diff_vw::TriggerCallback_, this);


}
diff_vw::~diff_vw()
{

}

void diff_vw::TriggerCallback_(const std_msgs::Int8& msg)
{
	  std::cout<<"diff_vw delete"<<std::endl;
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
void diff_vw::timerCallback(const ros::TimerEvent& event)
{
	Calculate_odom();
	std_msgs::Float32 msg;
	msg.data = v_buf;
	v_Publisher_.publish(msg);
	if(isReveice_joystick){
		v_buf = 0.04;

		switch(btn_id){
			case PUSE_BUTTON_A:
				joystick_move();
				break;

			case PUSE_BUTTON_X:
				joystick_move();
				break;

			case PUSE_BUTTON_Y:
				joystick_move();
				break;

			default:
				break;


		}

		isReveice_joystick = false;
	}
	else{

		LaserMiss += 1;

		if(LaserMiss > 5)
		{
			Stop();
			std_msgs::Int8 msg;
			msg.data = 6;
			Error_Publisher_.publish(msg);

		}
		else
		{
			State_Machine(p_state_);
		}
	}
}
void diff_vw::ClearCallback(const std_msgs::Int8& msg)
{
	int obs_clear_buf = msg.data;
	std::vector<SUB_MISSONPATH_SUBPOINT> zzz;

	switch (obs_clear_buf) {
		case Clear_Mode:

			int xxx;
			p_state_ = P_STATE_IDLE;
			if(A_misson.size() > 0)
				A_misson[ready_path_index].ALL_pathnode.clear();

			A_misson.clear();


			memset(&trafficGO_recv,0,sizeof(trafficGO_recv));

			now_A_misson=0;
			now_path_index=0;
			ready_path_index=0;
			Tracking_Angle_Init(xxx, true);
			Tracking_Trajectory(xxx, true);
			Trajectory_Tracking(xxx, true);
			Navigation_move(zzz,true);
			Misson_state(true);

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
			obs_return =false;          //????????????
			avoid_path.clear();

			protect_erase = true;
			traffic_send = false;


			break;
		case Close_AllObs:
			command_OBSMode = 1;

			break;
		case limitObs_Mode:
			command_OBSMode = 2;

			break;
		case Normal_Mode:
			command_OBSMode = 0;

			break;
		default:
			break;
	}


	std::cout<<"A_misson.size  clear"<<A_misson.size() <<std::endl;
}

void diff_vw::State_Machine(int& state)
{
int type,id;
	switch (state) {

		case P_STATE_IDLE:
			Command_STATE = Command_STATE_IDLE;

			Idle();

			break;
		case P_STATE_MISSON:
			Misson_state(false);


			break;

		case P_STATE_MOVE:
			//????????????????????????
			//????????????????????????id
			 type = A_misson[now_A_misson].sub_missonPath[now_path_index].end_type;
			 id = A_misson[now_A_misson].sub_missonPath[now_path_index].end;


			if(!traffic_send)
			{
				int id = A_misson[now_A_misson].sub_missonPath[now_path_index].end;
				Command_STATE = Command_STATE_TRACKING;
				Command_id = id;
			}

			if (trafficGO_recv.GO ==2) {
				if(type == Command_STATE_Traffic)
					Stop();
			}
			else
			{
				//??????
				Navigation_move(A_misson[now_A_misson].sub_missonPath, false);


				std::cout<<"type   "<<  type  <<std::endl;

				if(type == Command_STATE_Traffic)
				{
					if(traffic_send)
					{
						Command_STATE = Command_STATE_Traffic;
						//???????????????
						if(trafficGO_recv.id==id)
							if(trafficGO_recv.GO == 1)
							{

								A_misson[now_A_misson].sub_missonPath[now_path_index].start = A_misson[now_A_misson].sub_missonPath[now_path_index+1].start;
								A_misson[now_A_misson].sub_missonPath[now_path_index].end_type = A_misson[now_A_misson].sub_missonPath[now_path_index+1].end_type;
								A_misson[now_A_misson].sub_missonPath[now_path_index].end = A_misson[now_A_misson].sub_missonPath[now_path_index+1].end;
								A_misson[now_A_misson].sub_missonPath[now_path_index].sub_missonPath_subPoint.insert(A_misson[now_A_misson].sub_missonPath[now_path_index].sub_missonPath_subPoint.end(),A_misson[now_A_misson].sub_missonPath[now_path_index+1].sub_missonPath_subPoint.begin(),A_misson[now_A_misson].sub_missonPath[now_path_index+1].sub_missonPath_subPoint.end());
								A_misson[now_A_misson].sub_missonPath.erase(A_misson[now_A_misson].sub_missonPath.begin() + now_path_index + 1);
								trafficGO_recv.GO ==0;

								traffic_send = false;

							}
					}
				}
			}


			break;

	case P_STATE_STOP:

			Stop();

			break;


	case P_STATE_TIME_DELAY:  //LOADING TIME

			if(time_delay_counter >= time_delay_limit){
				time_delay_counter = 0;
				ROS_INFO(" Success!!!\n");
				p_state_ = P_STATE_MOVE;

				if(ready_path_index >= A_misson.size())
				{
					ready_path_index =0;
					A_misson[ready_path_index].ALL_pathnode.clear();
					A_misson.clear();
					p_state_=P_STATE_IDLE;
					isALLfinish =false;
					ROS_INFO("All Success!!!\n");
					now_A_misson = 0;

				}

			}
			else{
				time_delay_counter += 1;
				std::cout<<"time_delay_counter   "<<time_delay_counter<<std::endl;
			}

			break;

	case P_STATE_AVOID_WAIT:

			WaitObsLeave();

			break;

	case P_STATE_AVOID_REPLANNING:

			Stop();

			break;


	case P_STATE_AVOID_OBS:

			AvoidObs();

			break;


	case P_STATE_AVOID_DEADLOCK:

			WaitDeadlock();

			break;


	default:
			break;
}


}

bool diff_vw::Navigation_move(std::vector<SUB_MISSONPATH_SUBPOINT> & myAllSubPath, bool isReSet)
{

	static int path_index = 0;

	static int cnt = 0;

	float delay_time = 0.1;
	int cnt_limit = int(delay_time/time_sample);

	if(!isReSet){
		if(myAllSubPath.size() > 0){

			if(AvoidObs_state == P_OBS_NORMAL && isFindObs == true && cnt >= cnt_limit && !isCloseNow ){
				//?????????

				cnt = 0;
				Trajectory_Tracking(path_index, true);
				AvoidObs_state = P_OBS_WAIT;
				p_state_ = P_STATE_AVOID_WAIT;

			}
			else{

				if(isFindObs){
					cnt += 1;
				}
				else{
					cnt = 0;
				}

				bool isFInish = false;
				now_path_index=path_index;
				isFInish = Trajectory_Tracking(path_index, false);


				if(isFInish){

					path_index += 1;
					p_state_ = P_STATE_MISSON;

					if(path_index >= myAllSubPath.size()){
						cnt = 0;
						path_index = 0;

						global_v = 0;
						std::vector<unsigned char> command;
						//sendreceive.Package_Diff_encoder(0,0,command);
						sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);
						SendPackage(command);



						ready_path_index += 1;
						now_A_misson=ready_path_index -1;

						return true;

					}

				}
			}


		}
		else{
			ready_path_index=0;
			path_index = 0;
			p_state_ = P_STATE_IDLE;
			cnt = 0;
		}
	}
	else{
		ready_path_index=0;
		path_index = 0;
		Trajectory_Tracking(path_index, true);
		isALLfinish =false;
		cnt = 0;
		now_A_misson = 0;

	}
	last_subpath_index = path_index;
	return false;

}

void diff_vw::Idle()
{


	Navigation_move(A_misson[ready_path_index].sub_missonPath, true);

	global_v = 0;
	std::vector<unsigned char> command;

	// sendreceive.Package_Diff_encoder(0,0,
	// 		command);
	sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);

	SendPackage(command);
}

bool diff_vw::Trajectory_Tracking(int &subpath_index, bool isReSet)
{



	static bool isInitial = true;

	bool isFInish = false;

	bool isreset = isReSet;

	if(!isReSet){

		if(isInitial){

			isInitial = !(Tracking_Angle_Init(subpath_index, isreset));

		}
		else{

			isFInish = Tracking_Trajectory(subpath_index, isreset);

		}

		if(isFInish){
			Tracking_Angle_Init(subpath_index, true);
			Tracking_Trajectory(subpath_index, true);
			isInitial = true;
			return true;
		}


	}
	else{
		Tracking_Angle_Init(subpath_index, true);
		Tracking_Trajectory(subpath_index, true);
		isInitial = true;
	}

	return false;
}
bool diff_vw::Tracking_Angle_Init(int &subpath_index, bool isReSet)
{
	//return true;
	static bool isFInish = false;
	static bool isInitial = true;
	static int target_ind = 0;
	static float pre_angular_error = 0;


	float angular_kp = 0.5;
	float angular_kd = 0.1;
	float angular_ki = 0;

	float angular_error = 0;

	float Vx = 0;
	float Vy = 0;
	float W_rw = 0;

	static int cmd_angular_cnt = 0;
	int cmd_angular_cnt_limit = 10;


	std::string kinematics_model;

	Eigen::Vector3f target_pos;

	Eigen::Vector3f robot_pos = slam_pose_;

	if(!isReSet){

		if(isInitial){

			Command_id=A_misson[ready_path_index].sub_missonPath[subpath_index].end;

				isInitial = false;
				isCloseNow = true;


				int now_index = 0;
				float odom_v = 0.5;

				target_ind = calc_target_index(robot_pos, odom_v, A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint, now_index);

		}
		else{

			float cmd_angular_velocity = 0.0;
			float cmd_angular_velocity_buf = 0.0;


			target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind];

			float goal_angle = atan2((target_pos.y() - robot_pos.y()) , (target_pos.x() - robot_pos.x()));

			angular_error = goal_angle - robot_pos.z();


			if(angular_error>M_PI)
				angular_error=angular_error-2*M_PI;
			else if(angular_error<-M_PI)
				angular_error=angular_error+2*M_PI;

			float angular_p_error = angular_error;
			float angular_d_error = angular_error - pre_angular_error;
			pre_angular_error = angular_error;

			cmd_angular_velocity = angular_kp*angular_p_error + angular_kd*angular_d_error;



			if(fabs(cmd_angular_velocity) > stop_w_max){
				cmd_angular_velocity_buf = (cmd_angular_velocity/fabs(cmd_angular_velocity))*stop_w_max;
				cmd_angular_cnt += 1;
				if(cmd_angular_cnt > cmd_angular_cnt_limit)
				{
					cmd_angular_cnt = cmd_angular_cnt_limit;
					cmd_angular_velocity = cmd_angular_velocity_buf;
				}
				else
				{
					cmd_angular_velocity = cmd_angular_velocity_buf * cmd_angular_cnt /cmd_angular_cnt_limit;
				}

			}

			if(fabs(cmd_angular_velocity) < stop_w_min){
				cmd_angular_velocity = (cmd_angular_velocity/fabs(cmd_angular_velocity))*stop_w_min;
			}

			if(fabs(angular_error) < 0.04){
				isFInish = true;
				cmd_angular_velocity = 0;
			}

			Vx = 0;
			Vy = 0;

			W_rw = cmd_angular_velocity;

			Car.two_wheel_Kinematics(0, W_rw, vl,  vr );


			if(isFInish){
				vl = 0;
				vr = 0;
			}
		}

		global_v = 0;
		std::vector<unsigned char> command;

		// sendreceive.Package_Diff_encoder(0,W_rw,
		// 		command
		// 		);
		sendreceive.Package_publicWheel_encoder(0, 0, W_rw, 0, command);
		SendPackage(command);


		//        ROS_INFO("=====================================");
		ROS_INFO("Tracking_Angle");
		//        ROS_INFO("---------------");
		//        ROS_INFO("now: %f, %f", robot_pos.x(), robot_pos.y());
		//        ROS_INFO("target: %f, %f", target_pos.x(), robot_pos.y());
		//        ROS_INFO("angle_error: %f", angular_error);

		if(isFInish){
			isFInish = false;
			isInitial = true;
			target_ind = 0;
			pre_angular_error = 0;
			cmd_angular_cnt = 0;
			return true;
		}

	}
	else{
		isFInish = false;
		isInitial = true;
		target_ind = 0;
		pre_angular_error = 0;
		cmd_angular_cnt = 0;
	}

	return false;


}


bool diff_vw::Tracking_Trajectory(int &subpath_index, bool isReSet)
{
	static bool Endangle = false;
	static int target_ind = 0;
	static bool isFInish = false;
	static bool startmove = true;


	isCloseNow = false;


	float speedup_time_s = speedup * time_sample;
	float V_target = speed;//m/s

	//speed control
	static float pre_angular_velocity_error = 0;
	float angular_velocity_kp = tracking_kp;
	float angular_velocity_kd = tracking_kd;

	//position control
	static float pre_dis_error = 0;
	float v_kp = 0.2;
	float v_kd = 0.05;


	static float pre_angular_error = 0;
	float angular_kp = tracking_kp;
	float angular_kd = tracking_kd;

	float Vx = 0;
	float Vy = 0;
	float W_rw = 0;


	float cmd_velocity = 0.0;
	float cmd_angular_velocity = 0.0;
	float V_sub_target = 0.04;
	float dis_error = 1;
	float angular_error = 1;
	float angular_velocity_error=0;


	std::string kinematics_model;

	Eigen::Vector3f robot_pos, target_pos, obs_turn_pos;

	if(!isReSet){
		std::cout<<"=======Tracking_Trajectory======  "<<std::endl;

		//?????????????????? ???????????????????????????????????? ???????????????????????? ????????????
		if(trafficGO_recv.GO == 3)
		{
			V_target = trafficGO_recv.speed;
    	}

		robot_pos = slam_pose_;
		//robot_pos = ukf_pose_;

		int now_index;

		//???????????????rviz
		draw(TrackingLineMarker , 1.0, 1.0, 0.0, A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint);

		std::vector<Eigen::Vector3f> NULL_buf;
		draw(ALLOBSPointMarker , 1.0, 0.0, 0.0, NULL_buf);
		draw(AvoidLineMarker , 1.0, 0.0, 0.0, NULL_buf);
		draw(obs_line_pointMarker , 1.0, 1.0, 0.0, NULL_buf);

		drawLine(obs_LineMarker , 1.0, 0.0, 0.0, NULL_buf);
		drawLine(vertical_lineMarker , 1.0, 0.0, 0.0, NULL_buf);
		//???????????????rviz

		//????????????????????????id???
		int id = A_misson[ready_path_index].sub_missonPath[subpath_index].end;

		//????????????????????????????????????????????????
		for(int i=0;i<A_misson[ready_path_index].ALL_pathnode.size();i++){
			if(A_misson[ready_path_index].ALL_pathnode[i].id==id)
			{
				kinematics_model = A_misson[ready_path_index].ALL_pathnode[i].kin ;
				break;
			}
		}
		int type = A_misson[ready_path_index].sub_missonPath[subpath_index].end_type;

		//??????????????????????????????
		target_ind = calc_target_index(robot_pos, v_buf, A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint, now_index);
		//?????????
		target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind];

		//===if(startmove)===
		//???????????? ?????????????????????????????????
		//?????????????????????????????????????????????????????? ???????????????????????? ??????????????????????????????????????? ??????????????????????????????????????????????????????????????????????????????
		if(startmove)
		{
			//??????????????????????????? ???????????????????????????????????????????????????????????????????????????
			OBS_limilMode = true;

			float x_error_world = 0.0;
			float y_error_world = 0.0;
			float x_error_robot = 0.0;
			float y_error_robot = 0.0;
			//?????????????????????????????????????????????
			//?????? ???????????? +??? ????????????????????????
			Eigen::Vector3f finial_target_p ;//???????????????
			if(A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size() > target_ind + 3 )
			{
				finial_target_p << A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind + 3];

				x_error_world = finial_target_p.x() - target_pos.x();
				y_error_world = finial_target_p.y() - target_pos.y();
				x_error_robot = x_error_world*cos(-1*robot_pos.z()) - y_error_world*sin(-1*robot_pos.z());
				y_error_robot = x_error_world*sin(-1*robot_pos.z()) + y_error_world*cos(-1*robot_pos.z());
			}
			else//????????????????????????
			{
				finial_target_p << A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind -1];

				x_error_world = target_pos.x() - finial_target_p.x();
				y_error_world = target_pos.y() - finial_target_p.y();
				x_error_robot = x_error_world*cos(-1*robot_pos.z()) - y_error_world*sin(-1*robot_pos.z());
				y_error_robot = x_error_world*sin(-1*robot_pos.z()) + y_error_world*cos(-1*robot_pos.z());
			}
			//???????????????????????????????????????????????????????????????way_theta
			float way_theta = atan2(y_error_robot, x_error_robot);

			//?????????
			//??????????????????????????????
			obs_way_theta=way_theta;


			//?????????????????????????????????
			float m_error = 0.0;
			tra.closeline( finial_target_p, target_pos, robot_pos, m_error);

			//????????????????????????<5??????
			if(fabs(m_error) < 0.05 )
			{
				//????????????????????? ?????????????????????????????????????????????
				if(kinematics_model == "diff")
				{
					float world_theta = atan2(y_error_world, x_error_world);
					float th_error = world_theta - robot_pos.z();
					std::cout<<"th_error  "<<th_error *180/M_PI<<std::endl;
					if(fabs(th_error)< 0.05)
					{
						OBS_limilMode = false;
						startmove = false ;
					}
				}
				else
				{   OBS_limilMode = false;
					startmove = false ;
				}


			}
		}
		//===if(startmove)===



		//????????????????????????????????????????????????
		int obs_turn_int = calc_target_index_obsturn(robot_pos, v_buf, A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint, now_index);

		obs_turn_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[obs_turn_int];


		Eigen::Vector3f obs_turn_pos_close;
		if(now_index + predict_turn_dis / 0.04 < A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size())
			obs_turn_pos_close = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[now_index + predict_turn_dis / 0.04];
		else
			obs_turn_pos_close = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size()];




		//??????????????????????????????????????????obs_turn_pos_th
		float obs_turn_pos_x = 0;
		float obs_turn_pos_y = 0;
		tra.world2robot( obs_turn_pos, robot_pos, obs_turn_pos_x, obs_turn_pos_y);

		float obs_turn_pos_close_x = 0;
		float obs_turn_pos_close_y = 0;
		tra.world2robot( obs_turn_pos_close, robot_pos, obs_turn_pos_close_x, obs_turn_pos_close_y);


		float obs_turn_pos_th = atan2(obs_turn_pos_y,obs_turn_pos_x) - atan2(obs_turn_pos_close_y,obs_turn_pos_close_x);

		// std::cout<<"atan2(obs_turn_pos_y,obs_turn_pos_x) "<<atan2(obs_turn_pos_y,obs_turn_pos_x) * 180/M_PI<<std::endl;
		// std::cout<<"atan2(obs_turn_pos_close_y,obs_turn_pos_close_x) "<<atan2(obs_turn_pos_close_y,obs_turn_pos_close_x) * 180/M_PI<<std::endl;



		float obs_turn_pos_th_limit = (60.0*M_PI/180.0) - atan2(predict_turn_R,predict_turn_dis);

		// std::cout<<"obs_turn_pos_th_limit "<<obs_turn_pos_th_limit * 180/M_PI<<std::endl;
		 std::cout<<"obs_turn_pos_th "<<obs_turn_pos_th * 180/M_PI<<std::endl;


			//????????????
			//????????????23???????????????????????????
			if(fabs(obs_turn_pos_th) > obs_turn_pos_th_limit)
			{
				std::cout<<"========predict turn =========== "<<std::endl;
				//??????????????????
				OBS_limilMode = true;
				//V_sub_target = V_sub_target - 0.05*V_sub_target ;
				V_sub_target = V_sub_target - 0.0005*V_sub_target;

				//????????????????????????0.3
				if(V_sub_target < Min_nav_speed)
					V_sub_target = Min_nav_speed;

				//???????????????0??????????????????1????????????????????? ??????2:?????????????????????????????????????????????????????????????????? ?????????????????????????????? ??????????????????????????????0.3
				//???V_sub_target ??????
				if(v_buf < Min_nav_speed )
				{
					v_buf = v_buf + speedup_time_s;
					V_sub_target = v_buf;
					//ROS_INFO("v_buf=====A====: %f", v_buf);
				}
			}
			else//????????????????????????
			{
				bool ready_turn = false;
				//???????????????????????????
				 if(now_index > 5)//??????????????????????????? ??????????????????
				 {



					float dis_point = predict_turn_dis / 0.04;

					Eigen::Vector3f back_pose ;//????????????????????????
					Eigen::Vector3f front_pose ;//????????????????????????

					if(now_index + dis_point < A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size())
						front_pose = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[now_index + dis_point];
					else
						front_pose = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size()];

					if(now_index >= dis_point)
						back_pose = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[now_index - dis_point];
					else
						back_pose = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[0];

					float err_sinth = 0;

					Eigen::Vector3f vec1 = front_pose - robot_pos;
					Eigen::Vector3f vec2 = back_pose - robot_pos;

					Eigen::Vector3f vec_cross = vec1.cross(vec2);
					 					float vec1_dis = sqrt(pow(vec1.x(),2) + pow(vec1.y(),2) + pow(vec1.z(),2));
					 					float vec2_dis = sqrt(pow(vec2.x(),2) + pow(vec2.y(),2) + pow(vec2.z(),2));
					 					float vec_cross_dis = sqrt(pow(vec_cross.x(),2) + pow(vec_cross.y(),2) + pow(vec_cross.z(),2));
					 //??????????????????sinth
					 err_sinth = vec_cross_dis /(vec1_dis * vec2_dis);
										//????????????th ??????45??? ??????135??? ???????????????

					if(fabs(err_sinth) > 0.707 && fabs(obs_turn_pos_th) > 3.0 * M_PI/180.0)
					{
						std::cout<<"=*************NOW************= "<<std::endl;
						ready_turn = true;
						std::cout<<"err_sinth "<<err_sinth * 180/M_PI<<std::endl;
						OBS_limilMode = true;

						std::cout<<" err_sinth  V_sub_target "<<V_sub_target<<std::endl;

						V_sub_target = V_sub_target - 0.0005*V_sub_target ;


						if(V_sub_target < Min_nav_speed)
							V_sub_target = Min_nav_speed;
					}
					else
						OBS_limilMode = false;

				}

				//?????????????????????????????????

				//obs_return
				// if(!obs_return)
				// {

				//???????????? ????????????????????????
				if(!ready_turn)
				{
					std::cout<<" GO SPEED "<<V_sub_target<<std::endl;
					if(v_buf + speedup_time_s > V_target )
						V_sub_target = V_target;
					else
					{
						v_buf = v_buf + speedup_time_s;
						V_sub_target = v_buf;
						//ROS_INFO("v_buf=====B====: %f", v_buf);
					}
				}


				//}
			}







		int Path_size = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size();
		//===????????????====
		//?????????????????????
		//?????????30%?????????????????????
		int speed_down_count =(int)(Path_size*0.3);
		int speed_down_road = Path_size - speed_down_count;

		//===???85% ?????????====
		int First_tracking = Path_size * 0.2;
//std::cout<<"AA"<<std::endl;
		//3m ??????????????? 3m?????????????????? 1.5m?????????????????????
		if(Path_size > 40)
		{
			speed_down_count = 40;
			speed_down_road= Path_size - speed_down_count;
			First_tracking = Path_size - 25;
		}

		//?????????????????????
		//3m?????????????????? ?????? ?????????????????? ???????????????
		if(now_index > speed_down_road){
			V_sub_target = Min_nav_speed + (V_sub_target - Min_nav_speed)*double(speed_down_count - (now_index - speed_down_road))/double(speed_down_count);
			if(V_sub_target < Min_nav_speed)
				V_sub_target = Min_nav_speed;
			//std::cout<<"V_sub_target=====go state==="<< V_sub_target<<std::endl;
		}
		//===????????????====

		//===????????????====
		//?????????????????????????????????
		if(type == MISSON_traffic)
		{
			Eigen::Vector3f finial_traffic_p ;

			finial_traffic_p <<A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[Path_size - 1];

			float diserror_traffic = getDistance(finial_traffic_p.x(), finial_traffic_p.y(), robot_pos.x(), robot_pos.y());
			//????????????< traffic_dis ?????????????????? ???????????????????????????
			if(diserror_traffic < traffic_dis)
				traffic_send = true;
		}

		//===????????????====

		if(!Endangle)
		{
			Eigen::Vector3f NOW_P(0.0, 0.0, 0.0);
			Eigen::Vector3f NEXT_P(0.0, 0.0, 0.0);
			if(Path_size - 1 > now_index + 1)
			{
				NOW_P = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[now_index];
				NEXT_P = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[now_index + 1];

			}
			else
			{
				NOW_P = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[now_index];
				NEXT_P = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[now_index - 1];
			}

			std::fstream fout;
			fout.open("Real_cte.csv",std::fstream::app);

			float rl_cte = 0.0;
			rl_cte = getDistance(NOW_P.x(),NOW_P.y(),robot_pos.x(),robot_pos.y());
			fout<<rl_cte<<std::endl;


			std::fstream fout_robot;
			fout_robot.open("robot_pose.csv",std::fstream::app);
			fout_robot << robot_pos.x()<<","<<robot_pos.y()<<std::endl;
			fout_robot.close();

			std::fstream fout_robot_v;
			fout_robot_v.open("v.csv",std::fstream::app);
			fout_robot_v << Rev_V <<std::endl;
			fout_robot_v.close();
		}





		//===???80% ?????????====
	if(now_index < First_tracking ){
			//Angular velocity PID

			float  angular_velocity_p_error = 0.0, angular_velocity_d_error = 0.0;

			//PID????????????????????? ???????????????w ???????????????

				angular_velocity_error = atan2((target_pos.y() - robot_pos.y()) , (target_pos.x() - robot_pos.x())) - robot_pos.z();
				if(fabs(angular_velocity_error) > M_PI){
					if(angular_velocity_error > 0) angular_velocity_error = angular_velocity_error - 2*M_PI ;
					else                           angular_velocity_error = angular_velocity_error + 2*M_PI;
				}
				angular_velocity_p_error = angular_velocity_error;
				angular_velocity_d_error = angular_velocity_error - pre_angular_velocity_error;
				pre_angular_velocity_error = angular_velocity_error;
				cmd_angular_velocity = angular_velocity_kp*angular_velocity_p_error + angular_velocity_kd*angular_velocity_d_error;

				if(fabs(cmd_angular_velocity) >= move_w_max){
					if(cmd_angular_velocity > 0) cmd_angular_velocity = move_w_max;
					else cmd_angular_velocity = -1.0*move_w_max;
				}

				//w?????????????????????????????????????????????
				if(v_buf > Min_nav_speed )
					if(fabs(angular_velocity_error) > 0.1)//????????????w??????0.1?????? ???????????????
					{
						V_sub_target = V_sub_target - 0.0001*V_sub_target ;
						std::cout<<"========wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww=========== "<<std::endl;


						if(V_sub_target < Min_nav_speed)
							V_sub_target = Min_nav_speed;

					}




			//????????????????????? ???????????????XX????????? ???????????????????????????Min_nav_speed
			// if(V_sub_target > Min_nav_speed * 1.2 ){
			// 	V_sub_target = V_sub_target - V_sub_target * decay_obs_v;
			// }

				float way_theta = 0;
				if(!startmove)
				{
					float x_error_robot = 0.0;
					float y_error_robot = 0.0 ;
					tra.world2robot(target_pos,robot_pos,x_error_robot,y_error_robot);

					//?????????????????????????????? ?????????diff????????????????????????????????? ???????????????????????????????????????
					way_theta = atan2(y_error_robot, x_error_robot);

					//????????? ????????????0 (???????????????????????????)    ????????????way_theta (????????????????????????)
					obs_way_theta = 0;
					//obs_way_theta=way_theta;
				}

				//????????? V_sub_target == 0 ????????????INI ??????V_sub_target > 0 ???????????????
				// if(V_sub_target < Min_nav_speed && V_sub_target > 0)
				// {
				// 	if(fabs(cmd_angular_velocity) >= 0.2){
				// 		if(cmd_angular_velocity > 0) cmd_angular_velocity = 0.2;
				// 		else cmd_angular_velocity = -1*0.2;
				// 	}
				// }



				cmd_velocity = V_sub_target;
				Vx = cmd_velocity;
				Vy = 0;
				W_rw = cmd_angular_velocity;

				//???????????????????????????????????????
				//??????????????????diff????????????????????? ?????????omni?????? ???????????????????????? ?????????????????????????????????
				if(obs_return)
				{
					Eigen::Vector3f finial_target_p ;
					finial_target_p <<A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind - 3];

					float m_error = 0.0;
					tra.closeline( finial_target_p, target_pos, robot_pos, m_error);
					OBS_limilMode =true;
					if(fabs(m_error) < 0.08 )
					{
						obs_return = false ;
						OBS_limilMode = false;
					}
				}



				v_buf = V_sub_target;

			Car.two_wheel_Kinematics(cmd_velocity, W_rw, vl,  vr);


		}//===???80% ?????????====
		//===?????????====
else{

            robot_pos = slam_pose_;

            isCloseNow = true;

            target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[Path_size - 1];

            float x_error = target_pos.x() - robot_pos.x();
            float y_error = target_pos.y() - robot_pos.y();

            float x_error_robot = x_error*cos(-1*robot_pos.z()) - y_error*sin(-1*robot_pos.z());
            float y_error_robot = x_error*sin(-1*robot_pos.z()) + y_error*cos(-1*robot_pos.z());
            float way_theta = atan2(y_error_robot, x_error_robot);


            dis_error = sqrt(x_error*x_error + y_error*y_error);
            float dis_p_error = dis_error;
            float dis_d_error = dis_error - pre_dis_error;
            pre_dis_error = dis_error;

            cmd_velocity = v_kp*dis_p_error + v_kd*dis_d_error;
			cmd_velocity = 1.5*dis_p_error + 0.05*dis_d_error;

            if(fabs(cmd_velocity) > Min_nav_speed){
                    cmd_velocity = (cmd_velocity/fabs(cmd_velocity))*Min_nav_speed;
                }

			float LOWER_SPEED = 0.1;
            if(fabs(cmd_velocity) < LOWER_SPEED){
                    cmd_velocity = (cmd_velocity/fabs(cmd_velocity))*LOWER_SPEED;
            }




                float  angular_velocity_p_error = 0.0, angular_velocity_d_error = 0.0;

                angular_velocity_error = atan2((target_pos.y() - robot_pos.y()) , (target_pos.x() - robot_pos.x())) - robot_pos.z();
                if(fabs(angular_velocity_error) > M_PI){
                    if(angular_velocity_error > 0) angular_velocity_error = angular_velocity_error - 2*M_PI ;
                    else                           angular_velocity_error = angular_velocity_error + 2*M_PI;
                }
                angular_velocity_p_error = angular_velocity_error;
                angular_velocity_d_error = angular_velocity_error - pre_angular_velocity_error;
                pre_angular_velocity_error = angular_velocity_error;
                cmd_angular_velocity = angular_velocity_kp*angular_velocity_p_error + angular_velocity_kd*angular_velocity_d_error;


                W_rw = cmd_angular_velocity;

                if(fabs(W_rw)>move_w_max)
                {
                if(W_rw > 0) W_rw =move_w_max;
                else W_rw = -1.0*move_w_max;
                }

				Car.two_wheel_Kinematics(cmd_velocity, W_rw, vl,  vr);

        }

        int mode_rpm = 0;
        if(Endangle)
        {
			cmd_velocity = 0.0;
            std::cout<<"=========Endangle============="<<std::endl;
            angular_error = target_pos.z() - robot_pos.z();
            if(fabs(angular_error) > M_PI){
                if(angular_error > 0) angular_error = angular_error - 2*M_PI ;
                else                  angular_error = angular_error + 2*M_PI;
            }


            float angular_p_error = angular_error;
            float angular_d_error = angular_error - pre_angular_error;
            pre_angular_error = angular_error;
            cmd_angular_velocity = angular_kp*angular_p_error + angular_kd*angular_d_error;

            W_rw = cmd_angular_velocity;

            if(fabs(W_rw)>stop_w_max)
            {
                if(W_rw > 0) W_rw =stop_w_max;
                else W_rw = -1.0*stop_w_max;
            }

			Car.two_wheel_Kinematics(cmd_velocity, W_rw, vl,  vr );

            std::cout<<"angular_error  "<< angular_error <<std::endl;

            if(fabs(angular_error) <= 0.05)
            {
                isFInish = true;
				cmd_velocity = 0;
				W_rw = 0;
            }


        }

		if(dis_error <= 0.03 && Endangle == false)
        {
            Endangle = true;
			cmd_velocity = 0;
			W_rw = 0;
        }
		if(Endangle == true && fabs(angular_error) <= 0.05)
		{
			isFInish = true;
			cmd_velocity = 0;
			W_rw = 0;
		}

		if(type == MISSON_ChangeMode)
		{
			if(fabs(angular_error) <= 0.2 && dis_error <= 0.5){
				cmd_velocity = 0;
				W_rw = 0;
				isFInish = true;
				std::cout<<"isFInish A"<<std::endl;
			}

		}

		global_v = cmd_velocity;
		std::vector<unsigned char> command;

		// sendreceive.Package_Diff_encoder(cmd_velocity,W_rw,
		// 		command
		// 		);
		sendreceive.Package_publicWheel_encoder(cmd_velocity, 0, W_rw, 0, command);
		SendPackage(command);

		//        ROS_INFO("---------------");
		//        ROS_INFO("odom_v: %f", Rev_odom_v);
		//        ROS_INFO("now_index: %d", now_index);
		//        ROS_INFO("target_ind: %d", target_ind);
		//        ROS_INFO("---------------");
		//        ROS_INFO("Robot pose x: %f y: %f yaw: %f", slam_pose_[0], slam_pose_[1], slam_pose_[2]);
		//        ROS_INFO("---------------");
		//        ROS_INFO("Vx: %f", Vx);
		//        ROS_INFO("Vy: %f", Vy);
		//        ROS_INFO("W_rw: %f", W_rw);

		if(isFInish){
			Endangle  = false;
			isCloseNow = true;
			obs_return =false;
			target_ind = 0;
			startmove = true;
			isFInish = false;
			OBS_limilMode =false;
			pre_angular_velocity_error = 0;
			pre_dis_error = 0;
			pre_angular_error = 0;
			v_buf = 0.04;

			traffic_send = false;

			// sendreceive.Package_Diff_encoder(0,0,
			// 	command
			// 	);
			sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);
			SendPackage(command);
			return true;
		}
	}
	else{
		Endangle  = false;
		isCloseNow = true;
		obs_return =false;
		target_ind = 0;
		startmove = true;
		isFInish = false;
		OBS_limilMode =false;
		pre_angular_velocity_error = 0;
		pre_dis_error = 0;
		pre_angular_error = 0;
		v_buf = 0.04;
		traffic_send = false;
	}

	return false;


}




//==============OBS=================
void diff_vw::laserCallback(const sensor_msgs::LaserScan& scan)
{

	ros::Duration dur (0.5);

	LaserMiss = 0;
	//std::cout<<"ONE LASER"<<std::endl;
	//Receive LaserData
	int device_num = checkDeviceNum(scan);

	sensor_msgs::LaserScan device_scan_1;
	sensor_msgs::LaserScan device_scan_2;

	if(device_num == 1){

		device_scan_1.header          = scan.header;
		device_scan_1.angle_min       = scan.angle_min;
		device_scan_1.angle_max       = scan.angle_max;
		device_scan_1.angle_increment = scan.angle_increment;
		device_scan_1.time_increment  = scan.time_increment;
		device_scan_1.scan_time       = scan.scan_time;
		device_scan_1.range_min       = scan.range_min;
		device_scan_1.range_max       = scan.range_max;
		device_scan_1.ranges.resize(scan.ranges.size());
		device_scan_1.intensities.resize(scan.ranges.size());

		for(int i=0; i<scan.ranges.size(); i++)
			device_scan_1.ranges[i] = scan.ranges[i];

	}
	else{

		device_scan_1.header          = scan.header;
		device_scan_1.angle_min       = scan.angle_min;
		device_scan_1.angle_max       = scan.angle_max;
		device_scan_1.angle_increment = scan.angle_increment;
		device_scan_1.time_increment  = scan.time_increment;
		device_scan_1.scan_time       = scan.scan_time;
		device_scan_1.range_min       = scan.range_min;
		device_scan_1.range_max       = scan.range_max;
		device_scan_1.ranges.resize(scan.ranges.size()/2);
		device_scan_1.intensities.resize(scan.ranges.size()/2);

		device_scan_2.header          = scan.header;
		std::string frame_id_2_       = std::string("base_laser_link_2");
		device_scan_2.header.frame_id = frame_id_2_;
		device_scan_2.angle_min       = scan.angle_min;
		device_scan_2.angle_max       = scan.angle_max;
		device_scan_2.angle_increment = scan.angle_increment;
		device_scan_2.time_increment  = scan.time_increment;
		device_scan_2.scan_time       = scan.scan_time;
		device_scan_2.range_min       = scan.range_min;
		device_scan_2.range_max       = scan.range_max;
		device_scan_2.ranges.resize(scan.ranges.size()/2);
		device_scan_2.intensities.resize(scan.ranges.size()/2);


		for(int i=0; i<(scan.ranges.size()/2); i++){
			device_scan_1.ranges[i] = scan.ranges[i];
			device_scan_2.ranges[i] = scan.ranges[i+(scan.ranges.size()/2)];
		}
	}


	if (tf_.waitForTransform(p_base_frame_,device_scan_1.header.frame_id, device_scan_1.header.stamp,dur))
	{

		tf::StampedTransform laserTransform_1;

		tf_.lookupTransform(p_base_frame_,device_scan_1.header.frame_id, device_scan_1.header.stamp, laserTransform_1);

		if(device_num == 2)
		{
			if (tf_.waitForTransform(p_base_frame_,device_scan_2.header.frame_id, device_scan_2.header.stamp,dur))
			{
				tf::StampedTransform laserTransform_2;

				tf_.lookupTransform(p_base_frame_,device_scan_2.header.frame_id, device_scan_2.header.stamp, laserTransform_2);


				projector_1_.projectLaser(device_scan_1, laser_point_cloud_1_,20.0);

				projector_2_.projectLaser(device_scan_2, laser_point_cloud_2_,20.0);


				size_t laser_size_1 = laser_point_cloud_1_.points.size();

				size_t laser_size_2 = laser_point_cloud_2_.points.size();

				tf::Vector3 laserPos_1 (laserTransform_1.getOrigin());

				tf::Vector3 laserPos_2 (laserTransform_2.getOrigin());

				std::vector<Eigen::Vector3f> obs_container;


				std::vector<Eigen::Vector3f> obs_region_0; //-90 ~ -54
				std::vector<Eigen::Vector3f> obs_region_1; //-54 ~ -18
				std::vector<Eigen::Vector3f> obs_region_2; //-18 ~ 18
				std::vector<Eigen::Vector3f> obs_region_3; // 18 ~ 54
				std::vector<Eigen::Vector3f> obs_region_4; // 54 ~ 90


				for (size_t i = 0; i < laser_size_1; ++i){

					const geometry_msgs::Point32& currPoint(laser_point_cloud_1_.points[i]);

					float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

					if ( (dist_sqr > 0.05) && (dist_sqr < 20) ){

						if ( (currPoint.x < 0.0f) && (dist_sqr < 0.2f)){
							continue;
						}

						tf::Vector3 pointPosBaseFrame(laserTransform_1 * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

						double center_dis = sqrt(pointPosBaseFrame.x()*pointPosBaseFrame.x()+ pointPosBaseFrame.y()*pointPosBaseFrame.y());

						float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos_1.z();

						if (pointPosLaserFrameZ > -1 && pointPosLaserFrameZ < 1)
						{
							Eigen::Vector3f p(pointPosBaseFrame.x(), pointPosBaseFrame.y(), pointPosBaseFrame.z());
							obs_container.push_back(p);

						}
					}
				}


				for (size_t i = 0; i < laser_size_2; ++i)
				{

					const geometry_msgs::Point32& currPoint(laser_point_cloud_2_.points[i]);

					float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

					if ( (dist_sqr > 0.05) && (dist_sqr < 20) ){

						if ( (currPoint.x < 0.0f) && (dist_sqr < 0.2f)){
							continue;
						}

						tf::Vector3 pointPosBaseFrame(laserTransform_2 * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

						double center_dis = sqrt(pointPosBaseFrame.x()*pointPosBaseFrame.x()+ pointPosBaseFrame.y()*pointPosBaseFrame.y());

						float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos_2.z();

						if (pointPosLaserFrameZ > -1 && pointPosLaserFrameZ < 1)
						{

							Eigen::Vector3f p(pointPosBaseFrame.x(), pointPosBaseFrame.y(), pointPosBaseFrame.z());
							obs_container.push_back(p);

						}
					}

				}
				std::vector<Eigen::Vector3f> obs_gap_0;
				std::vector<Eigen::Vector3f> obs_gap_1;
				std::vector<Eigen::Vector3f> obs_gap_2;
				std::vector<Eigen::Vector3f> obs_gap_3;
				std::vector<Eigen::Vector3f> obs_gap_4;
				std::vector<Eigen::Vector3f> all_point;

				//--------------------------------------------//
				if(command_OBSMode != 1){
					obs_region_0.clear();
					obs_region_1.clear();
					obs_region_2.clear();
					obs_region_3.clear();
					obs_region_4.clear();

					// ObsDetect(obs_container, obs_region_0, obs_region_1, obs_region_2, obs_region_3, obs_region_4,obs_gap_0,obs_gap_1,obs_gap_2,obs_gap_3,obs_gap_4,all_point);

					// ObsStateMachine(obs_region_0, obs_region_1, obs_region_2, obs_region_3, obs_region_4,obs_gap_0,obs_gap_1,obs_gap_2,obs_gap_3,obs_gap_4,all_point);
					obs_container.clear();

				}

			}
		}
		else
		{std::cout<<"ONE LASER"<<std::endl;
			projector_1_.projectLaser(device_scan_1, laser_point_cloud_1_,20.0);

			size_t laser_size_1 = laser_point_cloud_1_.points.size();

			tf::Vector3 laserPos_1 (laserTransform_1.getOrigin());


			std::vector<Eigen::Vector3f> obs_container;


			std::vector<Eigen::Vector3f> obs_region_0; //-90 ~ -54
			std::vector<Eigen::Vector3f> obs_region_1; //-54 ~ -18
			std::vector<Eigen::Vector3f> obs_region_2; //-18 ~ 18
			std::vector<Eigen::Vector3f> obs_region_3; // 18 ~ 54
			std::vector<Eigen::Vector3f> obs_region_4; // 54 ~ 90

			Eigen::Vector3f robot_pos;
			robot_pos = slam_pose_;
			std::fstream fout;
			fout.open("robot_pos.xlsx",std::fstream::out);
			fout<<"===================" <<std::endl;
			fout<<robot_pos.x() <<"\t" <<robot_pos.y() <<"\t" <<robot_pos.z() <<std::endl;
			fout<<"===================" <<std::endl;



			for (size_t i = 0; i < laser_size_1; ++i){

				const geometry_msgs::Point32& currPoint(laser_point_cloud_1_.points[i]);

				float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

				if ( (dist_sqr > 0.05) && (dist_sqr < 20) ){

					if ( (currPoint.x < 0.0f) && (dist_sqr < 0.2f)){
						continue;
					}

					tf::Vector3 pointPosBaseFrame(laserTransform_1 * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

					double center_dis = sqrt(pointPosBaseFrame.x()*pointPosBaseFrame.x()+ pointPosBaseFrame.y()*pointPosBaseFrame.y());

					float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos_1.z();

					if (pointPosLaserFrameZ > -1 && pointPosLaserFrameZ < 1)
					{
						Eigen::Vector3f p(pointPosBaseFrame.x(), pointPosBaseFrame.y(), pointPosBaseFrame.z());
						obs_container.push_back(p);
						fout<<p.x() <<"\t" <<p.y()  <<std::endl;

					}
				}
			}
			fout.close();

			std::vector<Eigen::Vector3f> obs_gap_0;
			std::vector<Eigen::Vector3f> obs_gap_1;
			std::vector<Eigen::Vector3f> obs_gap_2;
			std::vector<Eigen::Vector3f> obs_gap_3;
			std::vector<Eigen::Vector3f> obs_gap_4;
			std::vector<Eigen::Vector3f> all_point;

			//--------------------------------------------//
			if(command_OBSMode != 1){
				obs_region_0.clear();
				obs_region_1.clear();
				obs_region_2.clear();
				obs_region_3.clear();
				obs_region_4.clear();

				// ObsDetect(obs_container, obs_region_0, obs_region_1, obs_region_2, obs_region_3, obs_region_4,obs_gap_0,obs_gap_1,obs_gap_2,obs_gap_3,obs_gap_4,all_point);

				// ObsStateMachine(obs_region_0, obs_region_1, obs_region_2, obs_region_3, obs_region_4,obs_gap_0,obs_gap_1,obs_gap_2,obs_gap_3,obs_gap_4,all_point);
				obs_container.clear();

			}

		}

	}


}
void diff_vw::ObsDetect(std::vector<Eigen::Vector3f> &i_obs_container,
		std::vector<Eigen::Vector3f> &i_obs_region_0,
		std::vector<Eigen::Vector3f> &i_obs_region_1,
		std::vector<Eigen::Vector3f> &i_obs_region_2,
		std::vector<Eigen::Vector3f> &i_obs_region_3,
		std::vector<Eigen::Vector3f> &i_obs_region_4,
		std::vector<Eigen::Vector3f> &i_obs_gap_0,
		std::vector<Eigen::Vector3f> &i_obs_gap_1,
		std::vector<Eigen::Vector3f> &i_obs_gap_2,
		std::vector<Eigen::Vector3f> &i_obs_gap_3,
		std::vector<Eigen::Vector3f> &i_obs_gap_4,
		std::vector<Eigen::Vector3f> &all_point)
{
	//????????????????????????

	std::vector<Eigen::Vector3f> i_obs_region_2_buf;


	int obs_num_limit = OBS_isObs;


	float car_warn_theta=fabs(obs_way_theta);
	if(car_warn_theta>M_PI/2)
		car_warn_theta=M_PI-car_warn_theta;


	float Car_Front = CarLength* cos(car_warn_theta)/2 +  CarWidth* sin(car_warn_theta)/2 ;
	float Car_Side = CarWidth * cos(car_warn_theta)/2 + CarLength * sin(car_warn_theta)/2;

	//??????????????????
	float protect_OBS_Side = Car_Side + OBS_Side;
	float protect_OBS_Front = Car_Front + OBS_Front;
	float protect_OBS_LimitFront = Car_Front + OBS_Front_limit;
	float protect_OBS_Front_avoid = Car_Front + (OBS_Front+ OBS_Front_limit)/2;


	if(obs_way_theta !=0)
		for(int i=0;i<i_obs_container.size();i++)
		{
			Eigen::Vector3f p_buf = i_obs_container[i];
			i_obs_container[i].x() = p_buf.x() *cos(-1*obs_way_theta) - p_buf.y() * sin(-1*obs_way_theta);
			i_obs_container[i].y() = p_buf.x() *sin(-1*obs_way_theta) + p_buf.y() * cos(-1*obs_way_theta);

		}

	for(int i=0; i<i_obs_container.size(); i++){
		Eigen::Vector3f p = i_obs_container[i];
		//??????????????????
		if(p.x() < protect_OBS_Front && p.x() > -1*Car_Front){
			if( fabs(p.y()) < OBS_SideMiss_Scale*protect_OBS_Side)
			{

				if(p.y() < -1*protect_OBS_Side && p.x() < Car_Front)       i_obs_region_4.push_back(p);
				else if(p.y() < -1*protect_OBS_Side && p.x() >= Car_Front)  i_obs_region_3.push_back(p);
				//else if(p.x()>0 &&  fabs(p.y()) <= protect_OBS_Side && !OBS_limilMode  ) i_obs_region_2.push_back(p);
				else if(p.x()>0 &&  fabs(p.y()) <= protect_OBS_Side )
				{
					if(OBS_limilMode || command_OBSMode==2)//???????????? ??? ????????????????????????????????????diff??? ????????????
					{

						if( p.x()<protect_OBS_LimitFront) i_obs_region_2.push_back(p);
						all_obs_dis = protect_OBS_LimitFront;
					}
					else if(isAvoidObs)// ?????????????????????????????????
					{

						if( p.x()< protect_OBS_Front_avoid) i_obs_region_2.push_back(p);
						all_obs_dis = protect_OBS_Front_avoid;
					}
					else
					{
						i_obs_region_2.push_back(p);
						all_obs_dis = protect_OBS_Front;

					}

				}
				else if(p.y() > protect_OBS_Side && p.x() >= Car_Front)  i_obs_region_1.push_back(p);
				else if(p.y() > protect_OBS_Side && p.x() < Car_Front)  i_obs_region_0.push_back(p);

			}
		}
		//??????????????????
		if(p.x() < (protect_OBS_Front *2) && p.x() > 0){
			if(fabs(p.y()) <= all_obs_dis   )i_obs_region_2_buf.push_back(p);
		}
		//gap????????????
		if(p.x() < OBS_LookGap && p.x() > 0.0)
		{
			if(fabs(p.y()) < OBS_LookGap)
			{
				if(p.y() < -1*protect_OBS_Side) i_obs_gap_4.push_back(p);
				else if (fabs(p.y()) <= protect_OBS_Side && p.x() < all_obs_dis * 2.0 ) i_obs_gap_2.push_back(p);
				else if (p.y() > protect_OBS_Side )  i_obs_gap_0.push_back(p);

			}

			if(p.x() > (Car_Front) )  all_point.push_back(p);
		}

	}

	if(i_obs_region_2_buf.size() >= obs_num_limit){
		decay_obs_v = OBS_Decay_V;
	}
	else
	{
		decay_obs_v = 0;
	}


	//???????????????rviz
	Eigen::Vector3f robot_pos;
	robot_pos = slam_pose_;
	float i_obs_region_2_buf_x = 0;
	float i_obs_region_2_buf_y = 0;
	std::vector<Eigen::Vector3f> look_front;
	for(int i=0;i<i_obs_region_2.size();i++)
	{
		tra.robot2world(i_obs_region_2[i],robot_pos,obs_way_theta,i_obs_region_2_buf_x,i_obs_region_2_buf_y);

		Eigen::Vector3f p ;
		p <<i_obs_region_2_buf_x , i_obs_region_2_buf_y, 1.0;
		look_front.push_back(p);
	}
	//???????????????rviz
	draw(OBSPointMarker , 0.0, 1.0, 0.0, look_front);
	//???????????????rviz


}

void diff_vw::ObsStateMachine(std::vector<Eigen::Vector3f> &i_obs_region_0,
		std::vector<Eigen::Vector3f> &i_obs_region_1,
		std::vector<Eigen::Vector3f> &i_obs_region_2,
		std::vector<Eigen::Vector3f> &i_obs_region_3,
		std::vector<Eigen::Vector3f> &i_obs_region_4,
		std::vector<Eigen::Vector3f> &i_obs_gap_0,
		std::vector<Eigen::Vector3f> &i_obs_gap_1,
		std::vector<Eigen::Vector3f> &i_obs_gap_2,
		std::vector<Eigen::Vector3f> &i_obs_gap_3,
		std::vector<Eigen::Vector3f> &i_obs_gap_4,
		std::vector<Eigen::Vector3f> &all_point)
{
	static int cnt = 0;
	int cnt_limit = 20;

	int obs_num_limit = OBS_isObs;

	if(AvoidObs_state == P_OBS_NORMAL){
		if(i_obs_region_2.size() > obs_num_limit){
			isFindObs = true;
		}
		else{
			isFindObs = false;
		}
	}
	else if(AvoidObs_state == P_OBS_WAIT){
		if(i_obs_region_2.size() > obs_num_limit){
			isFindObs = true;
		}
		else{
			isFindObs = false;
		}
	}
	else if(AvoidObs_state == P_OBS_REPLANNING){
		isFindObs = false;
		RePlanning(last_subpath_index, i_obs_gap_0, i_obs_gap_1, i_obs_gap_2, i_obs_gap_3, i_obs_gap_4, all_point);
	}
	else if(AvoidObs_state == P_OBS_AVOID){

		if(isblind){
			isFindObs = false;
			isAvoidObs = true;

		}
		else if(i_obs_region_2.size() >= obs_num_limit){
			isFindObs = true;
			isAvoidObs = false;
			cnt = 0;

		}
		else if(avoid_way < 0 &&
				i_obs_region_0.size() < obs_num_limit &&
				i_obs_region_1.size() < obs_num_limit){

			if(cnt >= cnt_limit){
				isFindObs = false;
				isAvoidObs = false;
				cnt = 0;

			}
			else{
				isFindObs = false;
				isAvoidObs = true;

				cnt ++;
			}


		}
		else if(avoid_way > 0 &&
				i_obs_region_3.size() < obs_num_limit &&
				i_obs_region_4.size() < obs_num_limit){
			std::cout<<"==============avoid_way >0==============  "<<std::endl;
			if(cnt >= cnt_limit){
				isFindObs = false;
				isAvoidObs = false;
				cnt = 0;

			}
			else{
				isFindObs = false;
				isAvoidObs = true;


				cnt ++;
			}
		}
		else{
			isFindObs = false;
			isAvoidObs = true;
			cnt = 0;
		}



	}
	else if(AvoidObs_state == P_OBS_DEADLOCK){
		if( i_obs_region_2.size() > obs_num_limit ){
			isFindObs = true;
		}
		else{
			isFindObs = false;
		}
	}


}
bool diff_vw::WaitObsLeave()
{

	static bool isInitial = true;
	static bool isStop = false;
	static int Acceleration_count = 0;

	static int delay_count_1 = 0;
	static int delay_count_2 = 0;

	static float start_v = 0;
	static float start_w = 0;
	static float decay_v = 0;


	float Acceleration_time = OBS_Stop_Sec; //s
	int Acceleration_count_limit = int(Acceleration_time/time_sample);


	float delay_time = OBS_Wait_Sec; //s
	int delay_count_limit = int(delay_time/time_sample);

	if(isInitial){
		start_v = global_v;
		decay_v = -1*start_v/Acceleration_count_limit;
		isInitial = false;
		isStop = false;
	}
	else{

		if(!isStop){


			Acceleration_count += 1;

			if(Acceleration_count < Acceleration_count_limit){
				start_v += decay_v;

			}
			else{
				start_v = 0;
				isStop = true;
				Acceleration_count = 0;
			}

		}
		else{

			if(isFindObs || isAvoidObs){

				delay_count_2 = 0;

				if(delay_count_1 < delay_count_limit){
					delay_count_1 += 1;
				}
				else{
					isInitial = true;
					isStop = false;
					Acceleration_count = 0;
					delay_count_1 = 0;
					delay_count_2 = 0;


					start_v = 0;
					start_w = 0;
					decay_v = 0;

					p_state_ = P_STATE_AVOID_REPLANNING;
					AvoidObs_state = P_OBS_REPLANNING;

					return true;
				}

			}
			else{

				delay_count_1 = 0;

				if(delay_count_2 < delay_count_limit){
					delay_count_2 += 1;
				}
				else{
					isInitial = true;
					isStop = false;
					Acceleration_count = 0;
					delay_count_1 = 0;
					delay_count_2 = 0;

					start_v = 0;
					start_w = 0;
					decay_v = 0;


					p_state_ = P_STATE_MOVE;
					AvoidObs_state = P_OBS_NORMAL;
					return true;
				}
			}

		}

		global_v = start_v;


		std::vector<unsigned char> command;


			// sendreceive.Package_Diff_encoder(start_v,start_w,
			// 		command
			// 		);
		sendreceive.Package_publicWheel_encoder(start_v, 0, start_w, 0, command);

		SendPackage(command);


	}

	return false;

}
void diff_vw::RePlanning(int &subpath_index,
		std::vector<Eigen::Vector3f> &i_obs_region_0,
		std::vector<Eigen::Vector3f> &i_obs_region_1,
		std::vector<Eigen::Vector3f> &i_obs_region_2,
		std::vector<Eigen::Vector3f> &i_obs_region_3,
		std::vector<Eigen::Vector3f> &i_obs_region_4,
		std::vector<Eigen::Vector3f> &all_point)
{


	ROS_INFO("-------------------");
	ROS_INFO("RePlanning");

	//??????????????????????????????????????????????????????????????????
	Eigen::Vector3f target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size()-1];
	Eigen::Vector3f robot_pos = slam_pose_;

	//???????????????????????????????????????????????????????????????????????????????????????
	Eigen::Vector3f Key_Point;

	float p_mobile_width = CarWidth;
	float p_mobile_length = CarLength;
	float dis_min = sqrt(p_mobile_width*p_mobile_width + p_mobile_length*p_mobile_length)/2.0;

	float x_error = target_pos.x() - robot_pos.x();
	float y_error = target_pos.y() - robot_pos.y();
	float dis_error = sqrt(x_error*x_error + y_error*y_error);

	bool isWorkable = false;

	if(dis_error > dis_min){

		int now_index = 0;
		float odom_v = 0.8;

		//?????????????????????
		int target_ind = calc_target_index_obs(robot_pos, odom_v, A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint, now_index);

		target_pos = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[target_ind];

		//??????????????????????????? ?????????????????????????????????
		if(obs_avoid_flag)
		{
			target_ind = calc_target_index_obs(robot_pos, v_buf, avoid_path, now_index);
			target_pos = avoid_path[target_ind];


		}

		isWorkable = FindKeyPoint(target_pos, Key_Point, i_obs_region_0, i_obs_region_2, i_obs_region_4 ,all_point);

		if(isWorkable){
			CreateNewPath(target_pos, Key_Point, subpath_index, avoid_path);

			ROS_INFO("----------------");
			//ROS_INFO("NEW target_pos: %f, %f", target_pos.x(), target_pos.y());
			ROS_INFO("NEW Key_Point: %f, %f", Key_Point.x(), Key_Point.y());


			p_state_ = P_STATE_AVOID_OBS;
			AvoidObs_state = P_OBS_AVOID;
			isblind = true;
			isAvoidObs = true;
			std::cout<<"==isblind==rep== "<<isblind<<std::endl;
		}
		else{
			//?????????key_point dead
			p_state_ = P_STATE_AVOID_DEADLOCK;
			AvoidObs_state = P_OBS_DEADLOCK;
			ROS_INFO(" DEAD");
		}

	}
	else{
		//?????????????????? ??????????????????dead
		p_state_ = P_STATE_AVOID_DEADLOCK;
		AvoidObs_state = P_OBS_DEADLOCK;
		ROS_INFO(" DEAD");
	}


	ROS_INFO("RePlanning OFF");

}

bool diff_vw::FindKeyPoint(Eigen::Vector3f& target, Eigen::Vector3f& key_p,
		std::vector<Eigen::Vector3f> &i_obs_region_0,
		std::vector<Eigen::Vector3f> &i_obs_region_2,
		std::vector<Eigen::Vector3f> &i_obs_region_4,
		std::vector<Eigen::Vector3f> &all_point)
{
	std::cout<<"find"<<std::endl;
	//??????????????????
	float p_mobile_width = CarWidth;
	float p_mobile_length = CarLength;

	//float scalar = 1.0;

	//????????????????????????????????????
	float gap_limit = p_mobile_width + OBS_Side;


	std::vector<float> gap_array;
	std::vector<Eigen::Vector3f> point_array;
	Eigen::Vector3f robot_pos = slam_pose_;


	Eigen::Vector3f target_robot;

	Eigen::Vector3f _robot;
	_robot<<0.0 ,0.0 ,1.0;
	Eigen::Vector3f move_line;
	Eigen::Vector3f obs_line;
	Eigen::Vector3f vertical_line;
	std::vector<Eigen::Vector3f> obs_line_point;



	bool flag_right = false;
	bool flag_left = false;
	bool isWorkable = false;

	int obs_num_limit = OBS_isObs;

	float x=0.0;
	float y=0.0;

	tra.world2robot(target, robot_pos,  x,  y);
	target_robot.x() = x;
	target_robot.y() = y;
	target_robot.z() = 1.0;


	move_line = _robot.cross(target_robot);
	std::cout<<"see 1 "<<std::endl;
	//??????????????????????????????????????????????????????
	//??????
	int count_dis =0;
	for(int i=0;i<i_obs_region_0.size() ;i++)
	{
		if(i_obs_region_0.size() >= obs_num_limit)
		{
			if(i_obs_region_0[i].x()< target_robot.x()/2)
			{
				float obs_dis = fabs(move_line.x() *i_obs_region_0[i].x() +move_line.y() *i_obs_region_0[i].y() +move_line.z())/sqrt(move_line.x() * move_line.x() +move_line.y()*move_line.y());
				if(obs_dis < OBS_Side*2)
				{
					count_dis++;
					std::cout<<"count_dis  ri "<<count_dis<<std::endl;
				}
			}
		}
	}
	if(count_dis > obs_num_limit)
	{
		flag_right = true;
	}
	std::cout<<"see 2 "<<std::endl;
	//??????
	count_dis =0;
	for(int i=0;i<i_obs_region_4.size() ;i++)
	{
		if(i_obs_region_4.size() >= obs_num_limit)
		{
			if(i_obs_region_4[i].x()< target_robot.x()/2)
			{
				float obs_dis = fabs(move_line.x() * i_obs_region_4[i].x() +move_line.y() *i_obs_region_4[i].y() +move_line.z())/sqrt(move_line.x() * move_line.x() +move_line.y()*move_line.y());
				if(obs_dis < OBS_Side*2)
				{
					count_dis++;
					std::cout<<"count_dis  le "<<count_dis<<std::endl;
				}
			}
		}
	}
	if(count_dis > obs_num_limit)
	{
		flag_left = true;
	}
	count_dis =0;


	std::cout<<"see 3 "<<std::endl;
	int tan_min_i =0 ;

	//?????????????????????
	if(!flag_left || !flag_right)
	{
		//float obs_dis_limit=10000.0;

		//?????????????????????????????????????????????
		float tan_min = 1.57;
		for(int i=0;i<i_obs_region_2.size() ;i++)
		{
			if(i_obs_region_2[i].x()> -1*p_mobile_length)
			{
				float tan = fabs(atan2(i_obs_region_2[i].y(),i_obs_region_2[i].x()));
				if(tan<tan_min)
				{
					tan_min = tan;
					tan_min_i=i;
				}
			}
		}


		//?????????????????????
		i_obs_region_2[tan_min_i].z()=1.0;
		Eigen::Vector3f obs_i_obs_region_2;
		obs_i_obs_region_2<<i_obs_region_2[tan_min_i].x() ,i_obs_region_2[tan_min_i].y()+0.1 ,1.0;
		obs_line = obs_i_obs_region_2.cross(i_obs_region_2[tan_min_i]);

		float scale = sqrt(obs_line.x() * obs_line.x() + obs_line.y() * obs_line.y());
		Eigen::Vector3f normalize_obs_line_Line;
		normalize_obs_line_Line << obs_line.x()/scale,obs_line.y()/scale,obs_line.z()/scale;

		//???obsline??????
		std::vector<Eigen::Vector3f> drawline_;
		Eigen::Vector3f linepoint;
		linepoint << 0, 0, 0;

		float y = 8.0;
		float x = (-normalize_obs_line_Line.z() - normalize_obs_line_Line.y()*y)/normalize_obs_line_Line.x();
		linepoint << x, y, 1.0;
		drawline_.push_back(linepoint);


		x = (-normalize_obs_line_Line.z() + normalize_obs_line_Line.y()*y)/normalize_obs_line_Line.x();
		linepoint << x, -y, 1.0;
		drawline_.push_back(linepoint);

		for(int i = 0;i<drawline_.size();i++)
		{
			tra.robot2world(drawline_[i],robot_pos,obs_way_theta,x,y);
			drawline_[i].x() = x;
			drawline_[i].y() = y;

		}

		drawLine( obs_LineMarker, 0.0, 1.0, 0.0, drawline_);
		//???obsline??????



		std::cout<<"see 4 "<<std::endl;
		//?????????????????????????????????????????????????????????????????????

		std::vector<Eigen::Vector3f> all_point_world;
		std::vector<Eigen::Vector3f> obs_line_point_world;

		for(int i=0;i<all_point.size() ;i++)
		{
			//if(all_point[i].x()> 0 && all_point[i].x() < all_obs_dis )
			{	all_point[i].z() = 1.0;
				//float obs_dis = fabs(obs_line.x() *all_point[i].x() +obs_line.y() *all_point[i].y() +obs_line.z())/sqrt(obs_line.x() * obs_line.x() +obs_line.y()*obs_line.y());
				float obs_dis = fabs(normalize_obs_line_Line.transpose() * all_point[i]);
				if(obs_dis < 0.6)
				{
					obs_line_point.push_back(all_point[i]);

				}

			}
			//???????????????rviz
			float all_point_buf_x = 0;
			float all_point_buf_y = 0;
			tra.robot2world(all_point[i],robot_pos,obs_way_theta,all_point_buf_x,all_point_buf_y);

			Eigen::Vector3f p ;
			p <<all_point_buf_x , all_point_buf_y, 1.0;
			all_point_world.push_back(p);


		}


		// //???????????????????????????
		Eigen::Vector3f vertical_p;
		//vertical_p <<robot_pos.x() ,robot_pos.y() ,1.0;
		vertical_p <<i_obs_region_2[tan_min_i].x()+0.1 ,i_obs_region_2[tan_min_i].y() ,1.0;
		vertical_line = vertical_p.cross(i_obs_region_2[tan_min_i]);

		scale = sqrt(vertical_line.x() * vertical_line.x() + vertical_line.y() * vertical_line.y());
		Eigen::Vector3f normalize_vertical_line;
		normalize_vertical_line << vertical_line.x()/scale,vertical_line.y()/scale,vertical_line.z()/scale;

		int right_cnt = 0;
		int left_cnt = 0;

		for(int i = 0;i <obs_line_point.size() ;i++)
		{
			obs_line_point[i].z() = 1.0;
			float obs_dis = normalize_vertical_line.transpose() * obs_line_point[i];
			if(obs_dis > 0)
				right_cnt +=1;
			else
				left_cnt +=1;
		}

		//??????????????????????????????????????????????????????????????????????????????
		std::cout<<"right_cnt  "<<right_cnt<<std::endl;
		std::cout<<"left_cnt  "<<left_cnt<<std::endl;
		if(right_cnt < 1)
		{
			Eigen::Vector3f obs_right;
			obs_right<<i_obs_region_2[tan_min_i].x() ,i_obs_region_2[tan_min_i].y()- 10.0 ,1.0;
			obs_line_point.push_back(obs_right);
		}
		if(left_cnt < 1)
		{
			Eigen::Vector3f obs_left;
			obs_left<<i_obs_region_2[tan_min_i].x() ,i_obs_region_2[tan_min_i].y()+ 10.0 ,1.0;
			obs_line_point.push_back(obs_left);
		}


		//???vertical_line??????
		std::vector<Eigen::Vector3f> drawline_vertical;
		Eigen::Vector3f linepoint_vertical;
		linepoint_vertical << 0, 0, 0;

		y = (-normalize_vertical_line.z()-normalize_vertical_line.z())/normalize_vertical_line.y();
		x = 5.0;
		linepoint_vertical << x, y, 1.0;
		drawline_vertical.push_back(linepoint_vertical);


		y = (-normalize_vertical_line.z()-normalize_vertical_line.z()*-1)/normalize_vertical_line.y();
		linepoint_vertical << -x, y, 1.0;
		drawline_vertical.push_back(linepoint_vertical);

		for(int i = 0;i<drawline_vertical.size();i++)
		{
			tra.robot2world(drawline_vertical[i],robot_pos,obs_way_theta,x,y);
			drawline_vertical[i].x() = x;
			drawline_vertical[i].y() = y;

		}

		drawLine( vertical_lineMarker, 0.0, 0.0, 1.0, drawline_vertical);


		for(int i = 0;i <obs_line_point.size() ;i++)
		{
			float obs_line_point_buf_x = 0;
			float obs_line_point_buf_y = 0;
			tra.robot2world(obs_line_point[i],robot_pos,obs_way_theta,obs_line_point_buf_x,obs_line_point_buf_y);

			Eigen::Vector3f p ;
			p <<obs_line_point_buf_x , obs_line_point_buf_y, 1.0;
			obs_line_point_world.push_back(p);
		}
		//draw(ALLOBSPointMarker , 0.0, 0.0, 1.0, all_point_world);
		draw(obs_line_pointMarker , 1.0, 1.0, 0.0, obs_line_point_world);
		//???????????????rviz





		std::cout<<"obs_line_point   "<< obs_line_point.size() <<std::endl;
		std::cout<<"see 5 "<<std::endl;
		//??????
		if(obs_line_point.size() > 1 )
			for(int i=0;i<obs_line_point.size()-1 ;i++)
			{
				for(int j=i+1;j<obs_line_point.size() ;j++)
				{
					if(obs_line_point[i].y() < obs_line_point[j].y())
					{
						Eigen::Vector3f buf;
						buf = obs_line_point[i];
						obs_line_point[i] = obs_line_point[j];
						obs_line_point[j] = buf;
						//std::cout<<"obs_line_point=============="<<std::endl;
					}
				}
			}


		std::cout<<"see 6 "<<std::endl;
		//??????
		float obs_dis=0.0;
		if(obs_line_point.size() > 1)
			for(int i=0;i<obs_line_point.size()-1 ;i++)
			{
				obs_dis = obs_line_point[i].y() - obs_line_point[i+1].y();
				if(obs_dis>=gap_limit)
				{
					gap_array.push_back(obs_dis);
					point_array.push_back(obs_line_point[i]);
					point_array.push_back(obs_line_point[i+1]);
				}

			}


	}


	std::cout<<"see 7 "<<std::endl;
	ROS_INFO("--------------");
	ROS_INFO("FindKeyPoint");
	for(int i=0; i<gap_array.size(); i++)
		ROS_INFO("gap_array: %f", gap_array[i]);
	ROS_INFO("gap_limit: %f", gap_limit);



	//?????????gap
	if(gap_array.size() > 0){


		Eigen::Vector3f obs_pos;

		int max_index = 0;
		float max_dis = gap_array[0];
		for(int i=1; i<gap_array.size(); i++){
			if(gap_array[i] > max_dis){
				max_index = i;
				max_dis = gap_array[i];
			}
		}

		ROS_INFO("max_dis: %f", max_dis);

		//gap?????????????????? ????????????
		if(max_dis >= gap_limit){
			//gap???i???point???i??????
			int index = max_index*2 + 1;
			Eigen::Vector3f p0 = point_array[index-1];
			Eigen::Vector3f p1 = point_array[index];
			Eigen::Vector3f p2 = Eigen::Vector3f( (p0.x()+p1.x())/2,  (p0.y()+p1.y())/2, 0.0);




			//??????????????????????????????????????????????????????????????????????????????????????????
			float region_theta = atan2(p2.y(), p2.x());
			if(region_theta >= 0) avoid_way = 1;
			else avoid_way = -1;

			// std::cout<<"avoid_way avoid_way  "<< avoid_way<<std::endl;

			// std::cout<<"region_theta  "<< region_theta<<std::endl;

			Eigen::Vector3f local_key_p = Eigen::Vector3f(0.0, 0.0, 0.0);
			float dx,dy;

			//???????????????????????? ????????????dx??????0

			if(avoid_way > 0){
				dx = (gap_limit/2.0)*cos(M_PI/2);
				dy = (gap_limit/2.0)*sin(M_PI/2);
				ROS_INFO("--------left--------");
				if(flag_left)
				{
					max_dis=0;
				}
				//???????????????????????????????????????0
				float p0_robot = p0.y();
				float p1_robot = p1.y();
				if(p0_robot<0 && p1_robot<0)
				{
					if(p0_robot > p1_robot)
					{
						obs_pos = p1;
					}
					else{
						obs_pos = p0;
					}
				}
				else{
					if(p0_robot < 0)
						obs_pos = p0;
					else
						obs_pos = p1;
				}
			}
			else{
				dx = (gap_limit/2.0)*cos(-1*M_PI/2);
				dy = (gap_limit/2.0)*sin(-1*M_PI/2);
				ROS_INFO("--------right--------");
				if(flag_right)
				{
					max_dis=0;
				}
				//???????????????????????????????????????0
				float p0_robot = p0.y();
				float p1_robot = p1.y();
				if(p0_robot>0 && p1_robot>0)
				{
					if(p0_robot > p1_robot)
					{
						obs_pos = p0;
					}
					else{
						obs_pos = p1;
					}
				}
				else{
					if(p0_robot > 0)
						obs_pos = p0;
					else
						obs_pos = p1;
				}
			}


			// ROS_INFO("----------------------");
			// ROS_INFO("p0: %f, %f", p0.x(), p0.y());
			// ROS_INFO("p1: %f, %f", p1.x(), p1.y());
			// ROS_INFO("p2: %f, %f", p2.x(), p2.y());
			// ROS_INFO("obs_pos: %f, %f", obs_pos.x(), obs_pos.y());
			// ROS_INFO("dx, dy: %f, %f", dx, dy);

			//?????????????????????????????????
			local_key_p[0] = obs_pos.x() + dx;
			local_key_p[1] = obs_pos.y() + dy;

			// ROS_INFO("local_key_p: %f, %f", local_key_p.x(), local_key_p.y());


			//??????????????????
			float obs_pos_world_x=0.0, obs_pos_world_y=0.0;
			tra.robot2world( local_key_p,  robot_pos,  obs_way_theta, dx, dy);
			tra.robot2world( obs_pos,  robot_pos,  obs_way_theta, obs_pos_world_x, obs_pos_world_y);

			key_p = Eigen::Vector3f(dx, dy, robot_pos.z()  + obs_way_theta);

			// std::cout<<"obs_pos_world_x  "<<obs_pos_world_x<<std::endl;
			// std::cout<<"obs_pos_world_y  "<<obs_pos_world_y<<std::endl;
			// std::cout<<"key_p  "<<key_p<<std::endl;
			// std::cout<<"robot_pos  "<<robot_pos<<std::endl;

			//??????gap???????????????????????????????????????????????????????????????????????????gap_limit??????

			float x_error_world = key_p.x() - obs_pos_world_x;
			float y_error_world = key_p.y() - obs_pos_world_y;
			//            float x_error_robot = x_error_world*cos(-1*robot_pos.z() - obs_way_theta) - y_error_world*sin(-1*robot_pos.z() - obs_way_theta);
			//            float y_error_robot = x_error_world*sin(-1*robot_pos.z() - obs_way_theta) + y_error_world*cos(-1*robot_pos.z() - obs_way_theta);

			float way_theta = atan2(y_error_world, x_error_world);
			//float warn_theta = fabs(way_theta - robot_pos.z());

			float warn_theta = (way_theta - robot_pos.z() + 2*M_PI) ;

			if(warn_theta > (2*M_PI)) warn_theta = warn_theta - 2*M_PI;

			if(warn_theta > M_PI) warn_theta = 2*M_PI - warn_theta;

			if(warn_theta>M_PI/2)
				warn_theta=M_PI-warn_theta;

			float Sca = OBS_SideMiss_Scale *0.9 ;
			if(Sca <1.2)
				Sca = 1.2;

			gap_limit = p_mobile_width * sin(warn_theta) + p_mobile_length * cos(warn_theta) + 2*OBS_Side *Sca;


			//std::cout<<"warn_theta  "<<warn_theta<<std::endl;

			if(max_dis >= gap_limit)
			{
				//??????????????????key_p??? ??????????????????????????????

				//???????????????????????? ????????????dx??????0
				if(avoid_way > 0){
					dx = (gap_limit/2.0)*cos(M_PI/2);
					dy = (gap_limit/2.0)*sin(M_PI/2);
					ROS_INFO("--------left--------");
				}
				else{
					dx = (gap_limit/2.0)*cos(-1*M_PI/2);
					dy = (gap_limit/2.0)*sin(-1*M_PI/2);
					ROS_INFO("--------right--------");
				}

				// std::cout<<"==================see==================="<<std::endl;
				// std::cout<<"gap_limit  "<<gap_limit<<std::endl;
				// std::cout<<"dx  "<<dx<<std::endl;
				// std::cout<<"dy  "<<dx<<std::endl;
				// std::cout<<"==================see==================="<<std::endl;

				//?????????????????????????????????
				local_key_p[0] = obs_pos.x() + dx;
				local_key_p[1] = obs_pos.y() + dy;

				//std::cout<<"local_key_p  "<<local_key_p<<std::endl;
				//??????????????????
				tra.robot2world( local_key_p,  robot_pos,  obs_way_theta, dx, dy);

				key_p = Eigen::Vector3f( dx, dy, 0.0);

				//std::cout<<"key_p  "<<key_p<<std::endl;

				isWorkable = true;
			}
			//=============change================

		}
	}


	obs_line_point.clear();
	gap_array.clear();
	point_array.clear();


	return isWorkable;


}
void diff_vw::CreateNewPath(Eigen::Vector3f& target, Eigen::Vector3f& key_point, int subpath_index, std::vector<Eigen::Vector3f> &new_path)
{

	float x_error = key_point.x() - target.x();
	float y_error = key_point.y() - target.y();


	// std::fstream fout;
	//     fout.open("avoid_LINE",std::fstream::out);
	if(obs_avoid_flag)
	{
		std::vector<Eigen::Vector3f> path;
		for(int i=0;i<avoid_path.size();i++){
		    Eigen::Vector3f path_buf;
		    path_buf.x() = avoid_path[i].x() + x_error;
		    path_buf.y() = avoid_path[i].y() + y_error;
		    path_buf.z() = 1.0;

		    path.push_back(path_buf);
		}
		avoid_path.clear();
		new_path = path;


		//?????????
		//???????????????rviz
		draw(AvoidLineMarker , 1.0, 0.1, 0.1, avoid_path);
		//???????????????rviz


	}
	else{

		for(int i=0;i<A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint.size();i++){
			Eigen::Vector3f path_buf;
			path_buf.x() = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[i].x() + x_error;
			path_buf.y() = A_misson[ready_path_index].sub_missonPath[subpath_index].sub_missonPath_subPoint[i].y() + y_error;
			path_buf.z() = 1.0;


			new_path.push_back(path_buf);
		}
		draw(AvoidLineMarker , 1.0, 0.1, 0.1, new_path);
	}




}
bool diff_vw::AvoidObs()
{

	//-----------------------------------
	float angular_velocity_kp = tracking_kp;
	float angular_velocity_kd = tracking_kd;
	float angular_velocity_error=0;
	float  angular_velocity_p_error = 0.0, angular_velocity_d_error = 0.0;
	float cmd_angular_velocity = 0.0;
	static float pre_angular_velocity_error = 0;


	static int target_ind = 0;
	static bool isInitial = true;
	float V_target = 0.2;//m/s
	float W_rw = 0;
	float cmd_velocity = 0.0;

	Eigen::Vector3f robot_pos, target_pos;

	if(isAvoidObs == true && isFindObs == false){

		robot_pos = slam_pose_;

		// std::fstream fout;
		// fout.open("obs_robot_pos",std::fstream::out);
		// fout<<robot_pos.x() <<"\t" <<robot_pos.y() <<std::endl;

		int now_index;


		target_ind = calc_target_index_obs(robot_pos, V_target, avoid_path, now_index);
		target_pos = avoid_path[target_ind];

		Eigen::Vector3f now_pos = avoid_path[now_index];


		float x_error_robot = 0.0 ;
		float y_error_robot = 0.0 ;
		tra.world2robot( target_pos,  robot_pos, x_error_robot, y_error_robot);


		// float way_theta = atan2(y_error_robot, x_error_robot);
		cmd_velocity = V_target;


		angular_velocity_error = atan2((target_pos.y() - robot_pos.y()) , (target_pos.x() - robot_pos.x())) - robot_pos.z();
				if(fabs(angular_velocity_error) > M_PI){
					if(angular_velocity_error > 0) angular_velocity_error = angular_velocity_error - 2*M_PI ;
					else                           angular_velocity_error = angular_velocity_error + 2*M_PI;
				}
				angular_velocity_p_error = angular_velocity_error;
				angular_velocity_d_error = angular_velocity_error - pre_angular_velocity_error;
				pre_angular_velocity_error = angular_velocity_error;
				cmd_angular_velocity = angular_velocity_kp*angular_velocity_p_error + angular_velocity_kd*angular_velocity_d_error;
		W_rw = cmd_angular_velocity;


		// static float time_count_avoid = 1;
		// float th_err = way_theta - Rev_odom_t1;


		//???????????????avoid?????????????????????
		if(isInitial){
			float m_error = 0.0;
			tra.closeline( now_pos, target_pos, robot_pos, m_error);


			if(fabs(m_error) < 0.03 )
			{

				isblind = false;
				isInitial = false;
				//time_count_avoid = 1;
			}
		}
		Car.two_wheel_Kinematics(cmd_velocity, W_rw, vl,  vr );

		std::vector<unsigned char> command;

		// sendreceive.Package_Diff_encoder(cmd_velocity,W_rw,
		// 	command);
	  sendreceive.Package_publicWheel_encoder(cmd_velocity, 0, W_rw, 0, command);
		SendPackage(command);





	}
	else if(isAvoidObs == false && isFindObs == true){
		target_ind = 0;
		isInitial = true;
		obs_avoid_flag =true;


		p_state_ = P_STATE_AVOID_REPLANNING;
		AvoidObs_state = P_OBS_REPLANNING;

		pre_angular_velocity_error = 0.0;



	}
	else if(isAvoidObs == false && isFindObs == false){
		target_ind = 0;
		isInitial = true;
		obs_avoid_flag =false;

		std::vector<unsigned char> command;


		// sendreceive.Package_Diff_encoder(0,0,
		// 	command);
		sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);

		SendPackage(command);

		p_state_ = P_STATE_MOVE;
		AvoidObs_state = P_OBS_NORMAL;
		obs_return = true;

		pre_angular_velocity_error = 0.0;


		//?????????
		//???????????????rviz
		std::vector<Eigen::Vector3f> NULL_buf;
		draw(AvoidLineMarker , 1.0, 0.0, 0.0, NULL_buf);
		//???????????????rviz
	}


	return false;


}

bool diff_vw::WaitDeadlock()
{

	static int cnt = 0;
	float delay_time = 2.0;
	int cnt_limit = int(delay_time/time_sample);


	static int cnt_re = 0;
	float delay_time_re = 2.0;
	int cnt_limit_re = int(delay_time_re/time_sample);


	static int cnt_error = 0;

	//???????????????????????????????????????
	if(!isFindObs){
		cnt += 1;

		if(cnt >= cnt_limit){
			cnt = 0;
			p_state_ = P_STATE_MOVE;
			AvoidObs_state = P_OBS_NORMAL;

			cnt_error = 0;

			return true;
		}
	}
	//??????????????????????????? ?????????????????????????????????
	else{
		cnt = 0;
		cnt_re +=1;
		if(cnt_re >= cnt_limit_re)
		{
			p_state_ = P_STATE_AVOID_WAIT;
			AvoidObs_state = P_OBS_WAIT;
			cnt_re = 0;
			cnt_error +=1;

			//?????????????????????                   ////??????error?????????????????????????????????
			if(cnt_error > 10)
			{
				std_msgs::Int8 msg;

				msg.data = 0;

				ReMissionState_Publisher_.publish(msg);
			}


		}
	}
	global_v = 0;
	std::vector<unsigned char> command;

	// sendreceive.Package_Diff_encoder(0,0,
	// 		command);
	sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);

	SendPackage(command);

	return false;
}
void diff_vw::joystickCallback(const move_robot::joystick& joystick)
{


	if(PUSE_BUTTON_RB != joystick.btn_id && PUSE_BUTTON_START!= joystick.btn_id)
		btn_id = joystick.btn_id;

	float vx = joystick.x * JOYSTICK_SCALAR;
	float vy = joystick.y * JOYSTICK_SCALAR;

	joystick_v = sqrt(pow(vy, 2) + pow(vx, 2));
    joystick_theta = atan2(vy, vx);

	isReveice_joystick = true;

}
void diff_vw::joystick_move()
{

	if(btn_id == PUSE_BUTTON_A)
	{
		//joystick_v = joystick_v/60;// 0.33m/s
		joystick_v = joystick_v;

		float us = joystick_theta;

		if(us>M_PI/2)
		{   us-=M_PI;
		}
		else if(us<-M_PI/2)
		{   us+=M_PI;
		}


		//L is the distance of both the wheel
		float L = LRWheeldis; //meter

		//W_rw
		float W_rw = (sin(us)/L)*joystick_v;


		if(fabs(W_rw) > move_w_max){
			if(W_rw > 0)
				W_rw = move_w_max;
			else
				W_rw = -1.0*move_w_max;
		}

		//==============change=============

		float V_avg = joystick_v;

		if(V_avg*cos(joystick_theta)<0)
		{
			V_avg = -1*V_avg;
		}

		if(fabs(V_avg) < 0.05)
			V_avg = 0;
		if(fabs(W_rw) < 0.02)
			W_rw = 0;



		float Vx = V_avg;
		float Vy = 0;

		Car.two_wheel_Kinematics(V_avg, W_rw, vl,  vr );

		global_v = V_avg;

		// kevin
		geometry_msgs::Twist cmd_vel_msg;
		cmd_vel_msg.linear.x = V_avg;
		cmd_vel_msg.linear.y = 0;
		cmd_vel_msg.linear.z = 0;
		cmd_vel_msg.angular.x = 0;
		cmd_vel_msg.angular.y = 0;
		cmd_vel_msg.angular.z = W_rw;
		cmd_vel_pub.publish(cmd_vel_msg);

		std::vector<unsigned char> command;

		// sendreceive.Package_Diff_encoder(V_avg,W_rw,
		// 		command
		// 		);
		sendreceive.Package_publicWheel_encoder(V_avg, 0, W_rw, 0, command);

		SendPackage(command);

		ROS_INFO("============diff================");



	}
	else if(btn_id == PUSE_BUTTON_X)
	{ROS_INFO("turn_left");

		Car.two_wheel_Kinematics(0, stop_w_max, vl,  vr );

		std::cout<<"vl  "<<vl<<std::endl;
		std::cout<<"vr  "<<vr<<std::endl;

		global_v = 0;

		std::vector<unsigned char> command;


			// sendreceive.Package_Diff_encoder(0,stop_w_max,
			// 		command
			// 		);
		sendreceive.Package_publicWheel_encoder(0, 0, stop_w_max, 0, command);

		SendPackage(command);

	}
	else if(btn_id == PUSE_BUTTON_Y)
	{ROS_INFO("turn_right");

		Car.two_wheel_Kinematics(0, -1.0*stop_w_max, vl,  vr );

		global_v = 0;

		std::vector<unsigned char> command;


			// sendreceive.Package_Diff_encoder(0,-1.0*stop_w_max,
			// 		command
			// 		);
		sendreceive.Package_publicWheel_encoder(0, 0, -1.0*stop_w_max, 0, command);

		SendPackage(command);

	}


}
void diff_vw::Calculate_odom()
{
	static float last_v = 0.0;
	Rev_a = (Rev_V - last_v)*10;

	geometry_msgs::PoseStamped odom_odometry;
	odom_odometry.header.frame_id = "base_link";
	odom_odometry.pose.position.x = Rev_V;
	odom_odometry.pose.position.y = 0;
	odom_odometry.pose.orientation.w = Rev_W;
	odom_odometry.pose.orientation.z = 0;
	odometry_Publisher_.publish(odom_odometry);

	std::cout<<"Rev_V = "<<Rev_V<<" "<<" Rev_W =  "<<Rev_W<<std::endl;

	last_v = Rev_V;




}

void diff_vw::RevProcess(double receive_period)
{
	int Receive_Package_Size = 256;

	ros::Rate r_receive(1.0 / receive_period);
    std::vector<unsigned char> rev_buf;
	while(1){
    //std::cout<<"=========LOSS encoder========="<<std::endl;
		if(mySerial.serial_ok == true){

			unsigned char buff[Receive_Package_Size];

			int readByte = 0;
			readByte = read(mySerial.fd,buff,11);

			if(readByte > 0){
			for(int i=0; i<readByte; i++)
			 		rev_buf.push_back(buff[i]);
					if(rev_buf[0] != 'E')
					{
						rev_buf.clear();
						//std::cout<<"=========LOSS encoder========="<<std::endl;
					}
			}
			if(rev_buf.size() == 11)
			{
				//sendreceive.RevProcess_two_wheel_encoder(rev_buf, Rev_V, Rev_W);
				sendreceive.RevProcess_publicWheel_encoder(rev_buf, Rev_V, Rev_th, Rev_W, type);
				rev_buf.clear();
		  }
		}

		r_receive.sleep();

	}

}

void diff_vw::Stop()
{

	//????????????????????????????????????????????????????????????

	global_v = 0;

	std::vector<unsigned char> command;

	// sendreceive.Package_Diff_encoder(0,0,
	// 				command
	// 				);
	sendreceive.Package_publicWheel_encoder(0, 0, 0, 0, command);
	SendPackage(command);

}
