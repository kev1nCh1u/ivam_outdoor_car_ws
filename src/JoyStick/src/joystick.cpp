#include "joystick.h"

Joystick::Joystick(char *joystick_dev)
{
	active = false;
	btn_id = 0;
	btn_last = 0;
	isPause = false;

	command_pos.x = 0.0;
	command_pos.y = 0.0;
	command_pos.r = 0.0;
	command_pos.x = 0.0;

	joystick_fd = 0;
	joystick_ev = new js_event();
	joystick_st = new joystick_state();
	joystick_fd = open(joystick_dev, O_RDONLY | O_NONBLOCK);
	if (joystick_fd > 0) {
		ioctl(joystick_fd, JSIOCGNAME(256), name);
		ioctl(joystick_fd, JSIOCGVERSION, &version);
		ioctl(joystick_fd, JSIOCGAXES, &axes);
		ioctl(joystick_fd, JSIOCGBUTTONS, &buttons);
		cout << "   Name: " << name << endl;
		cout << "Version: " << version << endl;
		cout << "   Axes: " << int(axes) << endl;
		cout << "Buttons: " << int(buttons) << endl;

		for(int i=0; i<axes; i++)
			joystick_st->axis.push_back(0);
		for(int i=0; i<buttons; i++)
			joystick_st->button.push_back(0);

		cout<<joystick_st->axis.size()<<endl;
		cout<<joystick_st->button.size()<<endl;
		active = true;
	}

	Point_data_publisher_ = nodeHandle_.advertise<JoyStick::joystick>("joystick", 10);


	receive_thread_ = new boost::thread(boost::bind(&Joystick::RevProcess, this, 0.001));
}

Joystick::~Joystick(){
	if (joystick_fd > 0) {
		active = false;
		close(joystick_fd);
	}
	delete joystick_st;
	delete joystick_ev;
	joystick_fd = 0;
}

void Joystick::readEv()
{


	ssize_t bytes = read(joystick_fd, joystick_ev, sizeof(*joystick_ev));
	if (bytes == sizeof(*joystick_ev)) {
		joystick_ev->type &= ~JS_EVENT_INIT;
		if (joystick_ev->type & JS_EVENT_BUTTON) {
			joystick_st->button[int(joystick_ev->number)] = joystick_ev->value;
		}
		if (joystick_ev->type & JS_EVENT_AXIS) {
			joystick_st->axis[int(joystick_ev->number)] = joystick_ev->value;
		}

		// std::cout<<"joystick_ev->type  "<< (int)joystick_ev->type <<std::endl;
		// std::cout<<"joystick_ev->number  "<< (int)joystick_ev->number <<std::endl;
		// std::cout<<"joystick_ev->value  "<< (int)joystick_ev->value <<std::endl;



		if(buttonPressed(PUSE_BUTTON_ID)){
			isPause = !(isPause);

			if(isPause)
				ROS_INFO("JoyStick Pause.");
			else
				ROS_INFO("JoyStick Start.");
		}
		if(buttonPressed(PUSE_BUTTON_A)){
			ROS_INFO("diff");
			btn_id = PUSE_BUTTON_A;
			btn_last = btn_id;

		}
		else if(buttonPressed(PUSE_BUTTON_B)){
			ROS_INFO("omni");
			btn_id = PUSE_BUTTON_B;
			btn_last = btn_id;
		}
		else if(buttonPressed(PUSE_BUTTON_X)){
			ROS_INFO("turn_left");
			btn_id = PUSE_BUTTON_X;
			btn_last = btn_id;
		}
		else if(buttonPressed(PUSE_BUTTON_Y)){
			ROS_INFO("turn_right");
			btn_id = PUSE_BUTTON_Y;
			btn_last = btn_id;
		}
		else if(buttonPressed(PUSE_BUTTON_RB)){
			ROS_INFO("send");
			btn_id = PUSE_BUTTON_RB;
		}
		else if(buttonPressed(PUSE_BUTTON_START)){
			ROS_INFO("Start node");
			btn_id = PUSE_BUTTON_START;
		}
		else if(buttonPressed(PUSE_BUTTON_BACK)){
			ROS_INFO("Wheel return node");
			btn_id = PUSE_BUTTON_BACK;
		}
		// else if((int)joystick_ev->type == 2 && (int)joystick_ev->number == 5 && btn_id != PUSE_BUTTON_RT)
		// {
		// 	ROS_INFO("RT");
		// 	btn_id = PUSE_BUTTON_RT;
		// 	std::vector<NODE_recv> NodeSet;

		// 	NODE_recv node_buf;
		// 		node_buf.id = 0;
		// 		node_buf.type = 0;
		// 		node_buf.id_pixel_x = 0.0;
		// 		node_buf.id_pixel_y = 0.0;
		// 		node_buf.id_heading = 0.0;
		// 		node_buf.kin = "diff";
		// 		node_buf.line = 0;
		// 		node_buf.radius = 100.0;


		// 		NodeSet.push_back(node_buf);
		// 		memset(&node_buf,0,sizeof(node_buf));


		// 		node_buf.id = 1;
		// 		node_buf.type = 2;
		// 		node_buf.id_pixel_x = 0.0;
		// 		node_buf.id_pixel_y = 0.0;
		// 		node_buf.id_heading = 0.0;
		// 		node_buf.kin = "diff";
		// 		node_buf.line = 0;
		// 		node_buf.radius = 100.0;

		// 		NodeSet.push_back(node_buf);
		// 		memset(&node_buf,0,sizeof(node_buf));


		// 	for(int i=0;i<NodeSet.size();i++)
		// 		{
		// 			std::cout<<"---------------" << std::endl;


		// 			float real_x=0;
		// 			float real_y=0;
		// 			float heading=0; //弳

		// 			real_x=NodeSet[i].id_pixel_x;
		// 			real_y=NodeSet[i].id_pixel_y;
		// 			heading=NodeSet[i].id_heading;

		// 			NodeSet[i].node_pose <<real_x,real_y,heading;


		// 			//=======把資料傳去move_robot
		// 			JoyStick::Node_recv msg;

		// 			msg.check=i+1;
		// 			msg.value =NodeSet.size();
		// 			msg.kin = NodeSet[i].kin;
		// 			msg.id =NodeSet[i].id;
		// 			msg.type =NodeSet[i].type;
		// 			msg.time =NodeSet[i].time;
		// 			msg.btn_finish =NodeSet[i].btn_finish;
		// 			msg.x=NodeSet[i].node_pose.x();
		// 			msg.y=NodeSet[i].node_pose.y();
		// 			msg.z=NodeSet[i].node_pose.z();
		// 			msg.line=NodeSet[i].line;
		// 			msg.radius=NodeSet[i].radius;

		// 			IdType_Publisher_.publish(msg);


		// 		}
		// 	NodeSet.clear();

		// }
		// else if((int)joystick_ev->type == 2 && (int)joystick_ev->number == 2 && btn_id != PUSE_BUTTON_LT)
		// {
		// 	ROS_INFO("LT");
		// 	btn_id = PUSE_BUTTON_LT;

		// 	std::vector<NODE_recv> NodeSet;

		// 	NODE_recv node_buf;
		// 		node_buf.id = 1;
		// 		node_buf.type = 0;
		// 		node_buf.id_pixel_x = 0.0;
		// 		node_buf.id_pixel_y = 0.0;
		// 		node_buf.id_heading = 0.0;
		// 		node_buf.kin = "diff";
		// 		node_buf.line = 0;
		// 		node_buf.radius = 100.0;


		// 		NodeSet.push_back(node_buf);
		// 		memset(&node_buf,0,sizeof(node_buf));


		// 		node_buf.id = 0;
		// 		node_buf.type = 2;
		// 		node_buf.id_pixel_x = 0.0;
		// 		node_buf.id_pixel_y = 0.0;
		// 		node_buf.id_heading = 0.0;
		// 		node_buf.kin = "diff";
		// 		node_buf.line = 0;
		// 		node_buf.radius = 100.0;

		// 		NodeSet.push_back(node_buf);
		// 		memset(&node_buf,0,sizeof(node_buf));


		// 	for(int i=0;i<NodeSet.size();i++)
		// 		{
		// 			std::cout<<"---------------" << std::endl;


		// 			float real_x=0;
		// 			float real_y=0;
		// 			float heading=0; //弳

		// 			real_x=NodeSet[i].id_pixel_x;
		// 			real_y=NodeSet[i].id_pixel_y;
		// 			heading=NodeSet[i].id_heading;

		// 			NodeSet[i].node_pose <<real_x,real_y,heading;


		// 			//=======把資料傳去move_robot
		// 			JoyStick::Node_recv msg;

		// 			msg.check=i+1;
		// 			msg.value =NodeSet.size();
		// 			msg.kin = NodeSet[i].kin;
		// 			msg.id =NodeSet[i].id;
		// 			msg.type =NodeSet[i].type;
		// 			msg.time =NodeSet[i].time;
		// 			msg.btn_finish =NodeSet[i].btn_finish;
		// 			msg.x=NodeSet[i].node_pose.x();
		// 			msg.y=NodeSet[i].node_pose.y();
		// 			msg.z=NodeSet[i].node_pose.z();
		// 			msg.line=NodeSet[i].line;
		// 			msg.radius=NodeSet[i].radius;

		// 			IdType_Publisher_.publish(msg);


		// 		}
		// 	NodeSet.clear();

		// }



		joystick_position pos_buf;

		command_pos = joystickPosition(0);




		//Debug
		// cout<<"=========================================="<<endl;
		// cout<<"---Button---"<<endl;
		// for(int i=0; i<joystick_st->button.size(); i++)
		//     cout<<joystick_st->button[i]<<" ";
		// cout<<endl;

		// cout<<"----Axis----"<<endl;
		// for(int i=0; i<joystick_st->axis.size(); i++){
		//    cout<<"Axis["<<i<<"]: "<<joystick_st->axis[i]<<endl;
		// }
	}
	//====================================================================================//
}

ssize_t Joystick::recvtimeout(int ss, js_event *buf, int timeout)
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


	return read(ss, buf, sizeof(*joystick_ev));
}

joystick_position Joystick::joystickPosition(int n) {

	joystick_position pos;

	if (n > -1 && n < axes) {
		int i0 = n*2, i1 = n*2+1;
		float x0 = joystick_st->axis[i0]/32767.0f, y0 = -joystick_st->axis[i1]/32767.0f;
		float x  = x0 * sqrt(1 - pow(y0, 2)/2.0f), y  = y0 * sqrt(1 - pow(x0, 2)/2.0f);

		pos.x = x0;
		pos.y = y0;

		pos.theta = atan2(y, x);
		pos.r = sqrt(pow(y, 2) + pow(x, 2));
	} else {
		pos.theta = pos.r = pos.x = pos.y = 0.0f;
	}
	return pos;
}

bool Joystick::buttonPressed(int n) {
	return n > -1 && n < buttons ? joystick_st->button[n] : 0;
}

bool Joystick::checkPause()
{

	return isPause;
}

void Joystick::RevProcess(double receive_period)
{
	int Receive_Package_Size = 32;

	ros::Rate r_receive(1.0 / receive_period);
	while(1){

		readEv();


		if(!(checkPause())){


			JoyStick::joystick msg;
			msg.btn_id =btn_id;
                        float x = command_pos.x * 0 + command_pos.y * 1;
                        float y = command_pos.x * -1 + command_pos.y * 0;
                        msg.x = x;
                        msg.y = y;
			msg.z = 0.0;

			Point_data_publisher_.publish(msg);


			// if((int)joystick_ev->number == 7)
			// {
			// 		if((int)joystick_ev->value < 0)
			// 		{
			// 				std::cout<<"up"<<std::endl;
			// 		}
			// 		else if((int)joystick_ev->value > 0)
			// 		{
			// 				std::cout<<"low"<<std::endl;
			// 		}
			// 		else
			// 		{
			// 				std::cout<<"no"<<std::endl;
			// 		}
			// }
			// else if((int)joystick_ev->number == 6)
			// {
			// 		if((int)joystick_ev->value < 0)
			// 		{
			// 				std::cout<<"left"<<std::endl;
			// 		}
			// 		else if((int)joystick_ev->value > 0)
			// 		{
			// 				std::cout<<"right"<<std::endl;
			// 		}
			// 		else
			// 		{
			// 				std::cout<<"no"<<std::endl;
			// 		}
			// }

			if(btn_id == PUSE_BUTTON_RB || btn_id == PUSE_BUTTON_START)
			{
				std::cout<<"btn_last "<<btn_last<<std::endl;
				btn_id = btn_last;
				std::cout<<"btn_id "<<btn_id<<std::endl;
			}
		}

		r_receive.sleep();

	}

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "JoyStick");
	ros::Time::init();


	Joystick *js;


	if(argc < 2){

		ROS_INFO("usage: <Joystick Device Name> ");

		return 0;

	}
	else{

		js = new Joystick(argv[1]);
		if(js->active == false){

			ROS_INFO("Open failed ");
			return 0;

		}

	}

	ROS_INFO("JoyStick Start.");
	ros::spin();

	return 0;
}
