#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>




using namespace cv;
using namespace std;
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

bool selectObject = false;
int trackObject = 0;

static Point origin;
static Point o_end;

static Rect selection;

int counter = 250;
int show_H = 0;
std_msgs::Int32MultiArray M_array;


Mat image;
double out_sec = 0;

static int initDestArea = 1; //Initialized to 1 to avoid DIV by 0 errors

void camShift(Mat inImg);

ros::Publisher pub_set_dynamixel;
ros::Publisher pub_set_step_Motor;
ros::Subscriber sub_step_Motor_position;

std_msgs::String set_dynamixel_msg;

long  M1_pos,M2_pos,M3_pos,M4_pos,M5_pos,M6_pos;


int mouse_mode = 0; 
int do_calibration = 0;

double move_x_scale,move_y_scale;
Point plot_minLoc;

int plot_mark_w,plot_mark_h;



void step_motor_position_callback(const std_msgs::Int32MultiArray::ConstPtr& array){
    
    ROS_INFO("position x =%d y= %d",array->data[0],array->data[1]);
    M1_pos = array->data[0];    M2_pos = array->data[1];    M3_pos = array->data[2];
    M4_pos = array->data[3];    M5_pos = array->data[4];    M6_pos = array->data[5];
    
    return;
    
}

static void onMouse( int event, int x, int y, int, void* )
{
    //ROS_INFO("Mouse x =%d y= %d",x,y);
    if(mouse_mode == 0){
            if( selectObject )
            {
                o_end = Point(x,y);
                selection.x = MIN(x, origin.x);
                selection.y = MIN(y, origin.y);
                selection.width = std::abs(x - origin.x);
                selection.height = std::abs(y - origin.y);

                selection &= Rect(0, 0, image.cols, image.rows);
                initDestArea = selection.area();//duo
            }

            switch( event )
            {
            case CV_EVENT_LBUTTONDOWN:
                origin = Point(x,y);
                o_end = Point(x,y);
        
                selection = Rect(x,y,0,0);
                selectObject = true;
                break;
            case CV_EVENT_LBUTTONUP:
                selectObject = false;
                if( selection.width > 0 && selection.height > 0 )
                { 
                trackObject = -1;
                Mat image_roi = image(selection);
                imwrite("/home/tori/Template.jpg",image_roi);
                }
                break;
            }
    }
    else if(mouse_mode == 1){
        
            switch( event )
            {
            case CV_EVENT_LBUTTONDOWN:
             
            int move_x ,move_y;
                
                move_x = (float)(x-(image.cols/2))/move_x_scale ;
                move_y = (float)(y-(image.rows/2))/move_y_scale ;
            
            M_array.data.clear();
            M_array.data.push_back((int)move_x);M_array.data.push_back((int)move_y);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
            M_array.data.push_back(2);
            pub_set_step_Motor.publish(M_array);
             
            break;
            case CV_EVENT_LBUTTONUP:
     
            break;
            }        
        
        
        
    }
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    camShift(cv_ptr->image);
    //   image_pub_.publish(cv_ptr->toImageMsg());

}

    
void calibration(){
 
    Mat result;
    Mat srcImg = image.clone();
    Mat template_img = imread("/home/tori/Template.jpg");
    
    matchTemplate(srcImg, template_img, result, CV_TM_SQDIFF);
    
    
    ros::Duration(1.5).sleep();    
    
    double minVal; 
    Point minLoc;
    minMaxLoc(result, &minVal, 0, &minLoc, 0);
    //rectangle(srcImg, minLoc, Point(minLoc.x+template_img.cols , minLoc.y+template_img.rows), cvScalar(255,0,0), 3);
    
    
}
int showHelp(Mat img,int co){
   
  Mat template_img = imread("/home/tori/Template.jpg");
  Mat result;
  
  matchTemplate(img, template_img, result, CV_TM_SQDIFF);

  double minVal; 
  Point minLoc;
  minMaxLoc(result, &minVal, 0, &minLoc, 0);
  rectangle(img, minLoc, Point(minLoc.x+template_img.cols , minLoc.y+template_img.rows), cvScalar(255,0,0), 3);
    
  
  //if(do_calibration)rectangle(img, plot_minLoc, Point(plot_minLoc.x+plot_mark_w , plot_minLoc.y+plot_mark_h), cvScalar(255,0,0), 3);
  
  int show_index = 0;  
  
  char buf[256]; sprintf(buf, "%d", co);
  putText(img,"Help" ,      Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL ,       1,cv::Scalar(0,200,100));

  putText(img,"Press s key: save to /home/tori/Npget.jpg " ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press b key: /go back" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press f key: /go front" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press k key: /set front" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press l key: /set back" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));

  sprintf(buf, "M1_pos: %ld",M1_pos);
  putText(img,buf,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  sprintf(buf, "M2_pos: %ld",M2_pos);
  putText(img,buf,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  sprintf(buf, "M3_pos: %ld",M3_pos);
  putText(img,buf,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  sprintf(buf, "M4_pos: %ld",M4_pos);
  putText(img,buf,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  sprintf(buf, "M5_pos: %ld",M5_pos);
  putText(img,buf,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  sprintf(buf, "M6_pos: %ld",M6_pos);
  putText(img,buf,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
    
  sprintf(buf, "%d", co);
  putText(img,buf   ,     Point(20,show_index+=30), FONT_HERSHEY_COMPLEX_SMALL ,        1,cv::Scalar(0,200,100));  

/*
  sprintf(buf, "%d , %d", minLoc.x,minLoc.y);
  putText(img,buf   ,     Point(20,show_index+=30), FONT_HERSHEY_COMPLEX_SMALL ,        1,cv::Scalar(0,200,100));  
  sprintf(buf, "%d , %d", template_img.cols,template_img.rows);
  putText(img,buf   ,     Point(20,show_index+=30), FONT_HERSHEY_COMPLEX_SMALL ,        1,cv::Scalar(0,200,100));  
  sprintf(buf, "%d , %d", result.cols,result.rows);
  putText(img,buf   ,     Point(20,show_index+=30), FONT_HERSHEY_COMPLEX_SMALL ,        1,cv::Scalar(0,200,100));  
  */
  sprintf(buf, "Press m key: Mouse mode = %d  ", mouse_mode);
  putText(img,buf   ,     Point(20,show_index+=30), FONT_HERSHEY_COMPLEX_SMALL ,        1,cv::Scalar(0,200,100));  
  
  
  sprintf(buf, "move_x_scale = %f ,move_y_scale = %f ", move_x_scale,move_y_scale);
  putText(img,buf   ,     Point(20,show_index+=30), FONT_HERSHEY_COMPLEX_SMALL ,        1,cv::Scalar(0,200,100));  
  
  
}

void camShift(Mat inImg)
{
  static Mat hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
  static bool paused = false;

  //If the image processing is not paused
  if( !paused )
  {
    if( inImg.empty() )
    {
      ROS_INFO("Camera image empty");
      return;//break;
    }
  }

  image = inImg;    // resize(inImg,image,Size(inImg.cols/2,inImg.rows/2),0,0,INTER_LINEAR);
 
  //Code to display an inverted image of the selected region
  //Remove this in the fall validation expt TBD
  if( selectObject && selection.width > 0 && selection.height > 0 )
  {
      cv::rectangle(image,cvPoint(origin.x,origin.y),cvPoint(o_end.x,o_end.y),cvScalar(255,0,0),2);
  }

    if(show_H){
        showHelp(image,counter--);
        if(counter<=0){
            show_H=0;counter=250;
        }
    }
  imshow( "CamShift Demo", image );

  char c = (char)waitKey(1);
  if( c == 27 )
      ROS_INFO("Exit boss");//break;
  switch(c)
  {
  case 'c':
   ROS_INFO("Press c key");//break;
   do_calibration = 1;
      break;
  case 'p':
    ROS_INFO("Press p key");//break;
      break;
  case 's':
  imwrite("/home/tori/Npget.jpg",image);
    ROS_INFO("Press s key save jpg");//break;
      break;
  case 'f':
    ROS_INFO("Press f key");//break;
    set_dynamixel_msg.data = "go_front";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case 'b':
    ROS_INFO("Press b key");//break;
    set_dynamixel_msg.data = "go_back";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case 'k':
    ROS_INFO("set front Press k key");//break;
    set_dynamixel_msg.data = "set_front";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case 'l':
    ROS_INFO("set back Press l key");//break;
    set_dynamixel_msg.data = "set_back";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;

  case 'h':
     M_array.data.clear();
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(1);
     pub_set_step_Motor.publish(M_array);
    show_H =1;
  break;
  
  case 'o':
     
     M_array.data.clear();
     M_array.data.push_back(0);M_array.data.push_back(100);M_array.data.push_back(0);
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(4);
     pub_set_step_Motor.publish(M_array);
  break;
  case 'i':
     
     M_array.data.clear();
     M_array.data.push_back(0);M_array.data.push_back(-100);M_array.data.push_back(0);
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(1);

     pub_set_step_Motor.publish(M_array);
  break;
  case 'u':
     
     M_array.data.clear();
     M_array.data.push_back(100);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(2);

     pub_set_step_Motor.publish(M_array);
  break;
  case 'y':
     
     M_array.data.clear();
     M_array.data.push_back(-100);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(3);

     pub_set_step_Motor.publish(M_array);
  break;
  
  case 'm':
      mouse_mode ^= 0x01;
  break;



  
  default:
      break;
  }

}



Mat cal_img_result,cal_img_src,cal_img_template;
Point cal_Loc_0,cal_Loc_1;

void Timer1(const ros::TimerEvent& event){
    
    static int go_step = 0;
    static int wait_sleep = 0;
    
    if(do_calibration){
        
      switch(go_step){
        case 0:
            cal_img_src = image.clone();
            cal_img_template = imread("/home/tori/Template.jpg");
            matchTemplate(cal_img_src, cal_img_template, cal_img_result, CV_TM_SQDIFF);
            minMaxLoc(cal_img_result, 0, 0, &cal_Loc_0, 0);

            plot_minLoc = cal_Loc_0;
            plot_mark_w = cal_img_template.cols;
            plot_mark_h = cal_img_template.rows;
        
            M_array.data.clear();
            M_array.data.push_back(100);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
            M_array.data.push_back(2);
            pub_set_step_Motor.publish(M_array);
            go_step = 1;
            break;
        case 1:
            wait_sleep++;
            if(wait_sleep > 20){
                go_step = 2;   //-- 2sec delay

            cal_img_src = image.clone();
            matchTemplate(cal_img_src, cal_img_template, cal_img_result, CV_TM_SQDIFF);
            minMaxLoc(cal_img_result, 0, 0, &cal_Loc_1, 0);

            move_x_scale = (cal_Loc_0.x - cal_Loc_1.x)/100.0;

            plot_minLoc = cal_Loc_1;
             
            }
        break;
        case 2:
            M_array.data.clear();
            M_array.data.push_back(0);M_array.data.push_back(100);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
            M_array.data.push_back(2);
            pub_set_step_Motor.publish(M_array);
            go_step = 3;
            wait_sleep=0;
            break;
        case 3:
            wait_sleep++;
            if(wait_sleep > 20){
                
                
                cal_img_src = image.clone();
                matchTemplate(cal_img_src, cal_img_template, cal_img_result, CV_TM_SQDIFF);
                minMaxLoc(cal_img_result, 0, 0, &cal_Loc_0, 0);

                move_y_scale = (cal_Loc_1.y - cal_Loc_0.y)/100.0;

                plot_minLoc = cal_Loc_0;


                
                go_step = 0;   //-- 2sec delay
                do_calibration = 0; 
                wait_sleep=0;
            }
        break;
            
      }
    }   
        
        
       // double move_x_scale,move_y_scale;
       /* Mat result;
        Mat srcImg = image.clone();
        Mat template_img = imread("/home/tori/Template.jpg");
    
        matchTemplate(srcImg, template_img, result, CV_TM_SQDIFF);
        double minVal; 
        Point minLoc;
        minMaxLoc(result, &minVal, 0, &minLoc, 0);
        */     
    
//    double secs =ros::Time::now().toSec();
//    out_sec = secs;
    //ROS_INFO("--------**---");//break;
}


/**
* This is ROS node to track the destination image
*/
int main(int argc, char **argv)
{
    move_x_scale = 1.0;
    move_y_scale = 1.0;
    
    ros::init(argc, argv, "image_processor");
//    ROS_INFO("-----------------");
    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);
	namedWindow( "CamShift Demo", CV_WINDOW_AUTOSIZE );
    /**
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used.
	* In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call
	* the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
	* subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe.
	* When the Subscriber object is destructed, it will automaticaInfoCallbacklly unsubscribe from the "camera/image_raw" base topic.
	*/
    
    //image_transport::Publisher image_pub_;
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);

    pub_set_dynamixel = nh.advertise<std_msgs::String>("set_dynamixel",100); //--  (,) 第一個參數為發布的話題第二個為發布序列的大小(緩存)，如果超過就丟棄
    pub_set_step_Motor = nh.advertise<std_msgs::Int32MultiArray>("step_Motor_control",100); //--  (,) 第一個參數為發布的話題第二個為發布序列的大小(緩存)，如果超過就丟棄
    sub_step_Motor_position = nh.subscribe<std_msgs::Int32MultiArray>("step_Motor_position", 100,step_motor_position_callback);//-- 訂閱設定訊息

    setMouseCallback( "CamShift Demo", onMouse, 0 );//

    
    
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), Timer1);

	/**
	* In this application all user callbacks will be called from within the ros::spin() call.
	* ros::spin() will not return until the node has been shutdown, either through a call
	* to ros::shutdown() or a Ctrl-C.
	*/
    ros::spin();
	//ROS_INFO is the replacement for printf/cout.
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
}

