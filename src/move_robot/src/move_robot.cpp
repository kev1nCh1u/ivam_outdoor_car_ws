#include "move_robot.h"
//#include "MPC_Diff_vw.h"
#include "Diff_vw.h"
#include "Diff_vw_A.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::Time::init();
    ros::NodeHandle now;
    ros::Subscriber TriggerSubscriber_=now.subscribe("Trigger", 5, TriggerCallback);
    int CarKind;		//車種()


    if(argc < 3){

        ROS_INFO("usage: [<Device name>] [<Baud rate>]");

        return 0;

    }



    int baudrate = std::atoi(argv[2]);

    LoadTitlePath();
    CarParameterPATH = TitlePath + CarParameterPATH_Local;

    int check = LoadCarKind(CarParameterPATH,CarKind);
    while(!check && ros::ok())
    {
        if(ReloadCarKind){
             LoadCarKind(CarParameterPATH,CarKind);
             break;
        }
        ros::spinOnce();
    }

    switch(CarKind){

      case 3:
               std::cout<<"CarKind = Diff_vw  "<<std::endl;
               diff_vw_A *Diff_vw_A;
               Diff_vw_A = new diff_vw_A(argv[1], baudrate);
          break;

        case 4:
                 std::cout<<"CarKind = Diff_vw  "<<std::endl;
                 diff_vw *Diff_vw;
                 Diff_vw = new diff_vw(argv[1], baudrate);
            break;

	    // case 5:
        //          std::cout<<"CarKind = MPC_DIFF_vw  "<<std::endl;
        //          mpc_diff_vw *MPC_Diff_vw;
        //          MPC_Diff_vw = new mpc_diff_vw(argv[1], baudrate);
        //     break;

            default:
            break;
    }

    ros::spin();

    return 0;


}
void TriggerCallback(const std_msgs::Int8& msg)
{
    int Trigger = msg.data;

    switch(Trigger){
            case ReloadCarParameter:
                 ReloadCarKind = true;
            break;

            default:
            break;

    }
}
bool LoadCarKind(std::string file_buf,int &returnbuf)
{

    std::fstream fin;

    char *file = const_cast<char *>(file_buf.c_str());
    fin.open(file, std::fstream::in);
    if(!fin.is_open())
    {
        ROS_INFO("Error: CarParameter is not opened!!");
        std::cout << "> " << file_buf.c_str() << std::endl;
        return false;
    }
    else{
        ROS_INFO("the file is opened!!");

        std::string s_CarName;
        std::getline(fin, s_CarName);
        std::getline(fin, s_CarName);
        //CarName = s_CarName;

        std::string s_Carnumber;
        std::getline(fin, s_Carnumber);
        std::getline(fin, s_Carnumber);
        //Carnumber = std::atoi(s_Carnumber.c_str());

        std::string s_CarKind;
        std::getline(fin, s_CarKind);
        std::getline(fin, s_CarKind);
        returnbuf = std::atoi(s_CarKind.c_str());
    }

    fin.close();
    return true ;


}
void LoadTitlePath()
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

	TitlePath = "/" + recv_pkg[1] + "/" + recv_pkg[2] + "/" + recv_pkg[3] + "/" + recv_pkg[4];
	std::cout<<"TitlePath  " <<TitlePath <<std::endl;

}
