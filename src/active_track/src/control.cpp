/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>


using namespace UNITREE_LEGGED_SDK;
int op=0,lr=0,bf=0;

void doMsg(const std_msgs::Int8::ConstPtr& msg_p){
    op = msg_p->data;
}


class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    printf("%d\n", op);
    ros::spinOnce();
    cmd.forwardSpeed = 0.0f;
    cmd.rotateSpeed = 0.0f;
    cmd.mode = 1;
    lr = op%10;
    bf = op/10;
    if(lr==1)
    {
    	cmd.mode = 2;
    	cmd.rotateSpeed = -0.3f;
	}
    else if(lr==2)
    {
    	cmd.mode = 2;
    	cmd.rotateSpeed = 0.3f;
	}
    if(bf==1)
    {
    	cmd.mode = 2;
    	cmd.forwardSpeed = -0.3f;
	}
    else if(bf==2)
	{
		cmd.mode = 2;
		cmd.forwardSpeed = 0.3f;  	
	}  
	

    udp.SetSend(cmd);
}

int main(int argc, char  *argv[]) 
{	//ros
	ros::init(argc,argv,"control");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<std_msgs::Int8>("detect_control",1,doMsg);
    
    
    //output information
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    custom.cmd.roll = 0.0f;
    custom.cmd.pitch = 0.0f;
    custom.cmd.yaw = 0.0f;
    custom.cmd.bodyHeight = 0.0f;
    custom.cmd.sideSpeed = 0.0f;
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
