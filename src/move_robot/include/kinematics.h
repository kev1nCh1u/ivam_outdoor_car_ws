#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>


class Car_Kinematics
{
public:
	void two_wheel_Kinematics(float V, float w,  float &vl, float &vr);
        void four_wheel_Kinematics_rpm(float Vx, float Vy, float W_rw, float Rev_odom_t1, float Rev_odom_t2, float Rev_odom_t3, float Rev_odom_t4, std::vector<int> &rpm, std::vector<int> &theta); //Vx Vy W_rw  vector順序 左前開始逆時針 ,theta 回傳+-1800
	void setCar_Par(float l_two, float wheel_r_four, float reduction_ratio_four, float wheel_half_length_four, float wheel_half_weight_four);
	void closeth(float &V, float &t, float Rev_odom_t);
	void closeth(float &t, float Rev_odom_t);
        


private: //（可用繼承 後面再改）

        //two_wheel
        float l;                        //輪距

        //four_wheel
        float wheel_r;                  //輪子半徑
        float reduction_ratio;          //減速比
        float wheel_half_length;        //輪距一半（前後）
        float wheel_half_weight;        //輪距一半（左右） 

	
};
void Car_Kinematics::setCar_Par(float l_two, float wheel_r_four, float reduction_ratio_four, float wheel_half_length_four, float wheel_half_weight_four)
{
        
        l = l_two;

        wheel_r = wheel_r_four;
        reduction_ratio = reduction_ratio_four;
        wheel_half_length = wheel_half_length_four;
        wheel_half_weight = wheel_half_weight_four;

}

void Car_Kinematics::Car_Kinematics::two_wheel_Kinematics(float V, float w,  float &vl, float &vr)
{
	vl = V - w*l/2;
	vr = V + w*l/2;
}
void Car_Kinematics::four_wheel_Kinematics_rpm(float Vx, float Vy, float W_rw, float Rev_odom_t1, float Rev_odom_t2, float Rev_odom_t3, float Rev_odom_t4, std::vector<int>&rpm, std::vector<int>&theta)
{

	float Vx1, Vx2, Vx3, Vx4, Vy1, Vy2, Vy3, Vy4, V1, V2, V3, V4;
    	float t1, t2, t3, t4;

	float x_w1 = wheel_half_length;
    	float y_w1 = wheel_half_weight;

    	float x_w2 = -1*wheel_half_length;
    	float y_w2 = wheel_half_weight;

    	float x_w3 = -1*wheel_half_length;
    	float y_w3 = -1*wheel_half_weight;

    	float x_w4 = wheel_half_length;
    	float y_w4 = -1*wheel_half_weight;

	//wheel 1
        Vx1 = Vx - y_w1*W_rw;
        Vy1 = Vy + x_w1*W_rw;
        V1 = sqrt(Vx1*Vx1 + Vy1*Vy1);
        t1 = atan2(Vy1, Vx1);
        //wheel 2
        Vx2 = Vx - y_w2*W_rw;
        Vy2 = Vy + x_w2*W_rw;
        V2 = sqrt(Vx2*Vx2 + Vy2*Vy2);
        t2 = atan2(Vy2, Vx2);
        //wheel 3
        Vx3 = Vx - y_w3*W_rw;
        Vy3 = Vy + x_w3*W_rw;
        V3 = sqrt(Vx3*Vx3 + Vy3*Vy3);
        t3 = atan2(Vy3, Vx3);
        //wheel 4
        Vx4 = Vx - y_w4*W_rw;
        Vy4 = Vy + x_w4*W_rw;
        V4 = sqrt(Vx4*Vx4 + Vy4*Vy4);
        t4 = atan2(Vy4, Vx4);


//================
// std::cout<<"t1 "<<t1<<std::endl;
// std::cout<<"t2 "<<t2<<std::endl;
// std::cout<<"t3 "<<t3<<std::endl;
// std::cout<<"t4 "<<t4<<std::endl;
	closeth(V1,t1,Rev_odom_t1);
	closeth(V2,t2,Rev_odom_t2);
	closeth(V3,t3,Rev_odom_t3);
	closeth(V4,t4,Rev_odom_t4);

// std::cout<<"Rev_odom_t1 "<<Rev_odom_t1<<std::endl;
// std::cout<<"Rev_odom_t2 "<<Rev_odom_t2<<std::endl;
// std::cout<<"Rev_odom_t3 "<<Rev_odom_t3<<std::endl;
// std::cout<<"Rev_odom_t4 "<<Rev_odom_t4<<std::endl;
		
		

        int FL_control_theta = int((t1*180/M_PI)*10);
        int BL_control_theta = int((t2*180/M_PI)*10);
        int FR_control_theta = int((t4*180/M_PI)*10);
        int BR_control_theta = int((t3*180/M_PI)*10);
                
        float FL_rpm = (V1/wheel_r)/(2*M_PI)*60*reduction_ratio;
        float BL_rpm = (V2/wheel_r)/(2*M_PI)*60*reduction_ratio;
        float FR_rpm = (V4/wheel_r)/(2*M_PI)*60*reduction_ratio;
        float BR_rpm = (V3/wheel_r)/(2*M_PI)*60*reduction_ratio;

        int FL_control_rpm = int(FL_rpm);
        int BL_control_rpm = int(BL_rpm);
        int FR_control_rpm = int(FR_rpm);
        int BR_control_rpm = int(BR_rpm);



	rpm[0] = FL_control_rpm;
	rpm[1] = BL_control_rpm;
	rpm[2] = BR_control_rpm;
	rpm[3] = FR_control_rpm;

	theta[0] = FL_control_theta;
	theta[1] = BL_control_theta;
	theta[2] = BR_control_theta;
	theta[3] = FR_control_theta;


}
void Car_Kinematics::closeth(float &V, float &t, float Rev_odom_t)
{
	float th1_normal = 0 ,th1_change = 0 ,errn_th1 = 0,errc_th1 = 0;
	th1_normal = t ;

	if(th1_normal >= 0.0)
		th1_change = th1_normal - M_PI;
	else 
		th1_change = th1_normal + M_PI;

	errn_th1 = fabs(th1_normal - Rev_odom_t);
	errc_th1 = fabs(th1_change - Rev_odom_t);

        if(errn_th1 > M_PI)
                errn_th1 = 2 * M_PI - errn_th1;
        else if(errc_th1 > M_PI)
                errc_th1 = 2 * M_PI - errc_th1;

	if(errn_th1 > errc_th1)
	{
		t = th1_change;
		V = V*-1;
	}
}
void Car_Kinematics::closeth(float &t, float Rev_odom_t)
{
	float th1_normal = 0 ,th1_change = 0 ,errn_th1 = 0,errc_th1 = 0;
	th1_normal = t ;

	if(th1_normal >= 0.0)
		th1_change = th1_normal - M_PI;
	else 
		th1_change = th1_normal + M_PI;

	errn_th1 = fabs(th1_normal - Rev_odom_t);
	errc_th1 = fabs(th1_change - Rev_odom_t);

        if(errn_th1 > M_PI)
                errn_th1 = 2 * M_PI - errn_th1;
        else if(errc_th1 > M_PI)
                errc_th1 = 2 * M_PI - errc_th1;

	if(errn_th1 > errc_th1)
	{
		t = th1_change;
	}
}
