#include "ros/ros.h"
#include "Leap.h"
#include "leap_motion/hand_data.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "hand_data");
    ros::NodeHandle nh("hand_data");
    ros::Publisher hand_data = nh.advertise<leap_motion::hand_data>("hand_id",1);
    leap_motion::hand_data msg;
    ros::Rate loop_rate(10);
    Leap::Controller controller;
    

    while(ros::ok()){
        Leap::Frame frame = controller.frame();
        Leap::FingerList fingersInFrame = frame.fingers();
std::cout<<"========="<<std::endl;
        for(Leap::FingerList::const_iterator fl = fingersInFrame.begin(); fl != fingersInFrame.end(); fl++){
            std::cout <<  *fl << std::endl;
            msg.hand_id = (*fl).type();
            hand_data.publish(msg);
        }

        loop_rate.sleep();


    }


}
