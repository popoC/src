#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

class TeleopStep
{
    
    public:
        TeleopStep();
        
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


TeleopStep::TeleopStep():
    linear_(1),
    angular_(2)
    {
   //     nh_.param("axis_linear", linear_, linear_);   //-- 初始化參數設定
        nh_.param("axis_linear", linear_, linear_);
        nh_.param("axis_angular", angular_, angular_);
        nh_.param("scale_angular", a_scale_, a_scale_);
        nh_.param("scale_linear",l_scale_,l_scale_);

        vel_pub_1 = nh_.advertise<std_msgs::Int16MultiArray>("step_Motor",100); //--  (,) 第一個參數為發布的話題第二個為發布序列的大小(緩存)，如果超過就丟棄
        
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &TeleopStep::joyCallback, this);//-- 訂閱搖桿訊息
        
        
        M_SPEED = 1;
        M1 = 0;M2 = 0;M3 = 0;M4 = 0;M5 = 0;M6 = 0;
        
    }

    

    

void TeleopStep::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    
    std_msgs::Int16MultiArray M_array;
    M_array.data.clear();

    
    if(joy->buttons[5] == 1)M_SPEED++;
    if(joy->buttons[4] == 1)M_SPEED--;
    if(M_SPEED>4)M_SPEED=4;
    if(M_SPEED<1)M_SPEED=1;
    
            //(joy->axes[6]) = 1    <-
            //(joy->axes[6]) = -1    -> 
            //(joy->axes[7]) = 1   方向 上
            //(joy->axes[7]) = -1  方向 下
            //(joy->buttons[0]) = 1    正方形
            //(joy->buttons[1]) = 1    X
            //(joy->buttons[2]) = 1    O
            //(joy->buttons[3]) = 1    三角形
            //(joy->buttons[4]) = 1    左上
            //(joy->buttons[5]) = 1    右上
            
            //(joy->axes[0]) = 1    <-o  左
            //(joy->axes[0]) = -1   o->  左
    
    /*if(joy->axes[0] !=0 ){ M1 = joy->axes[0]*10; M_SPEED = 1;}
    if(joy->axes[1] !=0 ){ M2 = joy->axes[1]*10; M_SPEED = 1;}        
    if(joy->axes[2] !=0 ){ M1 = joy->axes[2]*10; M_SPEED = 4;}
    if(joy->axes[5] !=0 ){ M2 = joy->axes[5]*10; M_SPEED = 4;}
     */
     M1 = joy->axes[0]*5;
     M2 = joy->axes[1]*5;
      
    M6 = joy->buttons[1] - (joy->buttons[3]);
    M4 = joy->axes[7];
    M5 = joy->axes[6];
    M3 = joy->buttons[2] - (joy->buttons[0]);

    
//    std_msgs::UInt16 button_press;
//    button_press.data = (joy->buttons[1]);// x
    M_array.data.push_back(M1); //-10~10
    M_array.data.push_back(M2); //-10~10
    M_array.data.push_back(M3); //-10~10
    M_array.data.push_back(M4); //-10~10
    M_array.data.push_back(M5); //-10~10
    M_array.data.push_back(M6); //-10~10
    M_array.data.push_back(M_SPEED); //-10~10
    M_array.data.push_back(joy->buttons[8]);
    M_array.data.push_back(joy->buttons[9]);
    M_array.data.push_back(joy->buttons[10]);
    
    vel_pub_1.publish(M_array);
    
    //ROS_INFO("ps4:[%d]:[%d]:[%d]:[%d]:[%d]:[%d]",_step_M1.data,_step_M2.data,_step_M3.data,_step_M4.data,_step_M5.data,_step_M6.data);
    //ros::Duration(0.5).sleep();
    
}
    
    
int main(int argc,char** argv)
{
    ros::init(argc,argv,"teleop_step_N");
    TeleopStep teleop_step;
    
    ros::spin();
    
    
}

/*
class TeleopStep
{
    
    public:
        TeleopStep();
        
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        
        ros::NodeHandle nh_;
        
        int linear_,angular_;
        double l_scale_, a_scale_;
        
        ros::Publisher vel_pub_1;
        ros::Publisher vel_pub_2;
        ros::Publisher vel_pub_3;
        ros::Publisher vel_pub_4;
        ros::Publisher vel_pub_5;
        ros::Publisher vel_pub_6;
        
        ros::Subscriber joy_sub_;
        
};


TeleopStep::TeleopStep():
    linear_(1),
    angular_(2)
    {
   //     nh_.param("axis_linear", linear_, linear_);   //-- 初始化參數設定
        nh_.param("axis_linear", linear_, linear_);
        nh_.param("axis_angular", angular_, angular_);
        nh_.param("scale_angular", a_scale_, a_scale_);
        nh_.param("scale_linear",l_scale_,l_scale_);
   
        vel_pub_1 = nh_.advertise<std_msgs::Int16>("step_M1", 1); //--  (,) 第一個參數為發布的話題第二個為發布序列的大小(緩存)，如果超過就丟棄
        vel_pub_2 = nh_.advertise<std_msgs::Int16>("step_M2", 1);
        vel_pub_3 = nh_.advertise<std_msgs::Int16>("step_M3", 1);
        vel_pub_4 = nh_.advertise<std_msgs::Int16>("step_M4", 1);
        vel_pub_5 = nh_.advertise<std_msgs::Int16>("step_M5", 1);
        vel_pub_6 = nh_.advertise<std_msgs::Int16>("step_M6", 1);
        
        //joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopStep::joyCallback, this);//-- 訂閱搖桿訊息
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopStep::joyCallback, this);//-- 訂閱搖桿訊息
    }

void TeleopStep::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    std_msgs::Int16 _step_M1;
    std_msgs::Int16 _step_M2;    
    std_msgs::Int16 _step_M3;    
    std_msgs::Int16 _step_M4;    
    std_msgs::Int16 _step_M5;    
    std_msgs::Int16 _step_M6;    
    
//    std_msgs::UInt16 button_press;
//    button_press.data = (joy->buttons[1]);// x
    _step_M1.data = (joy->axes[0])* 10; //-10~10
    _step_M2.data = (joy->axes[1])* 10; 
    _step_M3.data = (joy->axes[2])* 10; 
    _step_M4.data = (joy->axes[3]-1)* 10;  //- 上面左
    _step_M5.data = (joy->axes[4]-1)* 10;  //-  上右
    _step_M6.data = (joy->axes[5])* 10; 
    vel_pub_1.publish(_step_M1);
    vel_pub_2.publish(_step_M2);
    vel_pub_3.publish(_step_M3);
    vel_pub_4.publish(_step_M4);
    vel_pub_5.publish(_step_M5);
    vel_pub_6.publish(_step_M6);
    
    ROS_INFO("ps4:[%d]:[%d]:[%d]:[%d]:[%d]:[%d]",_step_M1.data,_step_M2.data,_step_M3.data,_step_M4.data,_step_M5.data,_step_M6.data);
    //ros::Duration(0.5).sleep();
    
}
    
    
int main(int argc,char** argv)
{
    ros::init(argc,argv,"teleop_step_N");
    TeleopStep teleop_step;
    
    ros::spin();
    
    
}
*/