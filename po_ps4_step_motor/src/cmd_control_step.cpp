#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

class CmdTeleopStep
{
    
    public:
        CmdTeleopStep();
        void run_motor();
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        
        ros::NodeHandle nh_;
        
        int linear_,angular_;
        double l_scale_, a_scale_;
        
        ros::Publisher vel_pub_1;
        
        ros::Subscriber joy_sub_;
        
        int M_SPEED;
        int M1,M2,M3,M4,M5,M6;
        
};


CmdTeleopStep::CmdTeleopStep():
    linear_(1),
    angular_(2)
    {

        vel_pub_1 = nh_.advertise<std_msgs::Int16MultiArray>("step_Motor",100); //--  (,) 第一個參數為發布的話題第二個為發布序列的大小(緩存)，如果超過就丟棄
        
        M_SPEED = 1;
        M1 = 0;M2 = 0;M3 = 0;M4 = 0;M5 = 0;M6 = 0;
        
    }

    

    
void CmdTeleopStep::run_motor(){
    
    std_msgs::Int16MultiArray M_array;
    int speed = 1;
    int time_cir = 10;
    
    for(int i=0;i<time_cir;i++){
        M_array.data.clear();
        M_array.data.push_back(1); M_array.data.push_back(1); M_array.data.push_back(0);
        M_array.data.push_back(0); M_array.data.push_back(0); M_array.data.push_back(0); 
        M_array.data.push_back(speed); M_array.data.push_back(0); M_array.data.push_back(0);
    
        vel_pub_1.publish(M_array);
        ros::Duration(0.1).sleep();
    }
    for(int i=0;i<time_cir;i++){
        M_array.data.clear();
        M_array.data.push_back(1); M_array.data.push_back(-1); M_array.data.push_back(0);
        M_array.data.push_back(0); M_array.data.push_back(0); M_array.data.push_back(0); 
        M_array.data.push_back(speed); M_array.data.push_back(0); M_array.data.push_back(0);
        vel_pub_1.publish(M_array);
        ros::Duration(0.1).sleep();
        //ROS_INFO("%d", i);
    }
    for(int i=0;i<time_cir;i++){
        M_array.data.clear();
        M_array.data.push_back(-1); M_array.data.push_back(-1); M_array.data.push_back(0);
        M_array.data.push_back(0); M_array.data.push_back(0); M_array.data.push_back(0); 
        M_array.data.push_back(speed); M_array.data.push_back(0); M_array.data.push_back(0);
    
        vel_pub_1.publish(M_array);
        ros::Duration(0.1).sleep();
    }
    for(int i=0;i<time_cir;i++){
        M_array.data.clear();
        M_array.data.push_back(-1); M_array.data.push_back(1); M_array.data.push_back(0);
        M_array.data.push_back(0); M_array.data.push_back(0); M_array.data.push_back(0); 
        M_array.data.push_back(speed); M_array.data.push_back(0); M_array.data.push_back(0);
        vel_pub_1.publish(M_array);
        ros::Duration(0.1).sleep();
        //ROS_INFO("%d", i);
    }
    
    M_array.data.clear();
    M_array.data.push_back(0); M_array.data.push_back(0); M_array.data.push_back(0);
    M_array.data.push_back(0); M_array.data.push_back(0); M_array.data.push_back(0); 
    M_array.data.push_back(1); M_array.data.push_back(0); M_array.data.push_back(0);
    
    vel_pub_1.publish(M_array);
    ROS_INFO("down");
    
}
    
int main(int argc,char** argv)
{
    ros::init(argc,argv,"cmd_teleop_step_N");
    CmdTeleopStep teleop_step;
    
    for(int j=0;j<2;j++){
        teleop_step.run_motor();
    }
    ros::spinOnce();
    
    
}
