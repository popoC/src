#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <sstream>

using namespace cv;
using namespace std;
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

bool selectObject = false;
static Point move_path;
static Point move_path_buff;



long Map_origin_x,Map_origin_y,Map_end_x,Map_end_y;

static Point origin;
static Point o_end;

static Rect selection;

int counter = 500;
int show_H = 0;
int show_Map = 0;

std_msgs::Int32MultiArray M_array;


Mat image;
Mat Map_image;

double out_sec = 0;

static int initDestArea = 1; //Initialized to 1 to avoid DIV by 0 errors

ros::Publisher pub_set_dynamixel;
ros::Publisher pub_set_step_Motor;
ros::Subscriber sub_step_Motor_position;

std_msgs::String set_dynamixel_msg;

long  M1_pos,M2_pos,M3_pos,M4_pos,M5_pos,M6_pos,M_Speed;


int mouse_mode = 0; 
int do_calibration = 0;

double move_x_scale,move_y_scale;
Point plot_minLoc;

int plot_mark_w,plot_mark_h;
FILE *fp;

void camShift(Mat inImg);
void updata_disp_set_config(int w_r);

void step_motor_position_callback(const std_msgs::Int32MultiArray::ConstPtr& array){
    
    ROS_INFO("position x =%d y= %d",array->data[0],array->data[1]);
    M1_pos = array->data[0];    M2_pos = array->data[1];    M3_pos = array->data[2];
    M4_pos = array->data[3];    M5_pos = array->data[4];    M6_pos = array->data[5];
    M_Speed = array->data[6];
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
                Mat image_roi = image(selection);
                imwrite("/home/po/Template.jpg",image_roi);
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
            M_array.data.push_back(4);
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
    Mat template_img = imread("/home/po/Template.jpg");
    
    matchTemplate(srcImg, template_img, result, CV_TM_SQDIFF);
    
    
    ros::Duration(1.5).sleep();    
    
    double minVal; 
    Point minLoc;
    minMaxLoc(result, &minVal, 0, &minLoc, 0);
    //rectangle(srcImg, minLoc, Point(minLoc.x+template_img.cols , minLoc.y+template_img.rows), cvScalar(255,0,0), 3);
    
    
}
int showHelp(Mat img,int co){
   /*
        Mat template_img = imread("/home/tori/Template.jpg");
        Mat result;
        matchTemplate(img, template_img, result, CV_TM_SQDIFF);

        double minVal;   Point minLoc;
        minMaxLoc(result, &minVal, 0, &minLoc, 0);
        rectangle(img, minLoc, Point(minLoc.x+template_img.cols , minLoc.y+template_img.rows), cvScalar(255,0,0), 3);
   */
    //if(do_calibration)rectangle(img, plot_minLoc, Point(plot_minLoc.x+plot_mark_w , plot_minLoc.y+plot_mark_h), cvScalar(255,0,0), 3);
  
  int show_index = 0;  
  
  char buf[256]; sprintf(buf, "%d", co);
  putText(img,"Demo Help" ,      Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL ,       1,cv::Scalar(0,200,100));
  putText(img,"Press ! key: /set Map origin point" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,0,20));
  putText(img,"Press @ key: /set Map end point" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,0,20));
  

  putText(img,"Press w key: save to /home/po/Npget.jpg " ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press b key: /go back" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press f key: /go front" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press s key: /go back2" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press x key: /go front2" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));

  putText(img,"Press a key: /run s2_up" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
  putText(img,"Press z key: /run s2_down" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));

  putText(img,"Press 1 key: /set back" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));
  putText(img,"Press 2 key: /set front" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));
  putText(img,"Press 3 key: /set back_m2" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));
  putText(img,"Press 4 key: /set front_m2" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));
  putText(img,"Press 5 key: /set back_down" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));
  putText(img,"Press 6 key: /set back_up" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));
  putText(img,"Press 7 key: /set front_down" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));
  putText(img,"Press 9 key: /auto run" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));
  putText(img,"Press 0 key: /go bottom" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(200,50,100));

  putText(img,"Press q key: /show map" ,Point(20,show_index+=30),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));

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
  sprintf(buf, "M_Speed: %ld",M_Speed);
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
  
  sprintf(buf, "map_x_y = %ld,%ld,%ld,%ld ",Map_origin_x,Map_origin_y,Map_end_x,Map_end_y);
  putText(img,buf   ,     Point(20,show_index+=30), FONT_HERSHEY_COMPLEX_SMALL ,        1,cv::Scalar(0,200,100));  


  sprintf(buf, "map_point_x_y = %d,%d ", move_path.x,move_path.y);
  putText(img,buf   ,     Point(20,show_index+=30), FONT_HERSHEY_COMPLEX_SMALL ,        1,cv::Scalar(0,200,100));  


  line(img,cvPoint(img.cols/2,0),cvPoint(img.cols/2,img.rows) ,CV_RGB(255,0,0),2,CV_AA,0);
  line(img,cvPoint(0,img.rows/2),cvPoint(img.cols,img.rows/2) ,CV_RGB(255,0,0),2,CV_AA,0);

}

void camShift(Mat inImg)
{
  //static Mat hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
  static bool paused = false;
  
  std::stringstream stringstream_ss;

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
            show_H=0;counter=500;
        }
    }

  if(show_Map){


      move_path.x = (float)(M1_pos - Map_origin_x ) / (Map_end_x - Map_origin_x) *Map_image.cols;
      move_path.y = (float)(M2_pos - Map_origin_y ) / (Map_end_y - Map_origin_y) *Map_image.rows;
    if(move_path.x < Map_image.cols && move_path.y < Map_image.rows){
        //cv::line(Map_image,move_path, cvPoint(move_path.x,move_path.y) ,cvScalar(0,0,255),5 );
       
       if(move_path_buff.x!=-1 && move_path_buff.y!=-1)cv::line(Map_image,move_path, move_path_buff ,cvScalar(0,0,255),5 );
        
       move_path_buff = move_path;
    }
     

    // Map_image = Mat::zeros(200, 200, CV_8UC3), 
   
   // Map_image.at<Vec3b>(move_path.x,move_path.y)[0] = 10;   // Map_image.at<Vec3b>(move_path.x,move_path.y)[1] = 10;   // Map_image.at<Vec3b>(move_path.x,move_path.y)[2] = 230;

/*
       move_path.y ++;
    if(move_path.y >=200){
        move_path.x +=10;
        if(move_path.x >=200)
        {
            move_path.x=0;
        }
        move_path.y =0;
    }

*/
    
     
    Mat imageROI;
    imageROI = image(Rect(image.cols-200,0,Map_image.cols,Map_image.rows));
    addWeighted(imageROI,0.5,Map_image,0.5,0.,imageROI);
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
  case 'w':
      time_t t;
      t = time(0);
      char file_name_buff[80];
      struct tm * now;
      now = localtime(&t);

      strftime(file_name_buff,80,"/home/po/image/%m_%d_%H_%M_%S.jpg", now);

      imwrite(file_name_buff,image);


      putText(image,file_name_buff,Point(image.cols/2,image.rows/2-10),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
      imshow( "CamShift Demo", image );
      //putText(image,"file_name_buff",Point(image.cols/3,image.rows/3-10),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
      if(cvWaitKey(3000)=='y'){  

      }
      //ROS_INFO("Press s key save jpg");//break;
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
  case 's':
    ROS_INFO("Press s key");//break;
    set_dynamixel_msg.data = "go_front_m2";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case 'x':
    ROS_INFO("Press x key");//break;
    set_dynamixel_msg.data = "go_back_m2";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;

  case '1':
    ROS_INFO("set back Press 1 key");//break;
    set_dynamixel_msg.data = "set_back";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case '2':
    ROS_INFO("set front Press 2 key");//break;
    set_dynamixel_msg.data = "set_front";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case '3':
    ROS_INFO("set back_m2 Press 3 key");//break;
    set_dynamixel_msg.data = "set_back_m2";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case '4':
    ROS_INFO("set front_m2 Press 4 key");//break;
    set_dynamixel_msg.data = "set_front_m2";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case '5':
    stringstream_ss.str("");
    stringstream_ss << M5_pos;

    ROS_INFO("set back_down Press 5 key");//break;
    set_dynamixel_msg.data = "set_back_down = "+stringstream_ss.str();
    
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case '6':
    stringstream_ss.str("");
    stringstream_ss << M5_pos;
  
    ROS_INFO("set back_up Press 6 key");//break;
    set_dynamixel_msg.data = "set_back_up = "+stringstream_ss.str();
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case '7':
    stringstream_ss.str("");
    stringstream_ss << M5_pos;
  
    ROS_INFO("set front_down Press 7 key");//break;
    set_dynamixel_msg.data = "set_front_down = "+stringstream_ss.str();
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case '9':
    putText(image,"auto run -- y or n" ,Point(image.cols/2,image.rows/2-10),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
    imshow( "CamShift Demo", image );
    if(cvWaitKey(0)=='y'){  
        set_dynamixel_msg.data = "go_go";
        pub_set_dynamixel.publish(set_dynamixel_msg);      
    }
  break;
  case '0':
    putText(image,"go bottom -- y or n" ,Point(image.cols/2,image.rows/2-10),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
    imshow( "CamShift Demo", image );
    if(cvWaitKey(0)=='y'){  
        set_dynamixel_msg.data = "go_bottom";
        pub_set_dynamixel.publish(set_dynamixel_msg);      
    }
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
     M_array.data.push_back(1);
     pub_set_step_Motor.publish(M_array);
  break;
  case 'i':
     
     M_array.data.clear();
     M_array.data.push_back(0);M_array.data.push_back(-100);M_array.data.push_back(0);
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(2);

     pub_set_step_Motor.publish(M_array);
  break;
  case 'u':
     
     M_array.data.clear();
     M_array.data.push_back(100);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(4);

     pub_set_step_Motor.publish(M_array);
  break;
  case 'y':
     
     M_array.data.clear();
     M_array.data.push_back(-100);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(8);

     pub_set_step_Motor.publish(M_array);
  break;
  case 't':
     
     M_array.data.clear();
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(350);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(8);

     pub_set_step_Motor.publish(M_array);
  break;
  case 'g':
     
     M_array.data.clear();
     M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(-350);M_array.data.push_back(0);M_array.data.push_back(0);
     M_array.data.push_back(8);

     pub_set_step_Motor.publish(M_array);
  break;
  
  case 'm':
      putText(image," y or n" ,Point(image.cols/2,image.rows/2-10),  FONT_HERSHEY_COMPLEX_SMALL , 1,cv::Scalar(0,200,100));
      imshow( "CamShift Demo", image );
      if(cvWaitKey(0)=='y'){  
            mouse_mode ^= 0x01;
      }
  break;

  case 'z':
    set_dynamixel_msg.data = "m2_go+";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;
  case 'a':
    set_dynamixel_msg.data = "m2_go-";
    pub_set_dynamixel.publish(set_dynamixel_msg);      
  break;

  case 'q':
            show_Map ^= 0x01;
           
  break;

  case '!':
            Map_origin_x = M1_pos;
            Map_origin_y = M2_pos;
            updata_disp_set_config(2);
  break;
  case '@':
            Map_end_x = M1_pos;
            Map_end_y = M2_pos;
            updata_disp_set_config(2);
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
            cal_img_template = imread("/home/po/Template.jpg");
            matchTemplate(cal_img_src, cal_img_template, cal_img_result, CV_TM_SQDIFF);
            minMaxLoc(cal_img_result, 0, 0, &cal_Loc_0, 0);

            plot_minLoc = cal_Loc_0;
            plot_mark_w = cal_img_template.cols;
            plot_mark_h = cal_img_template.rows;
        
            M_array.data.clear();
            M_array.data.push_back(100);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);M_array.data.push_back(0);
            M_array.data.push_back(4);
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
            M_array.data.push_back(4);
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
            
                updata_disp_set_config(2);
                /*
                fp = fopen("/home/po/disp_GUI_set_conf.txt","w");
                fprintf(fp,"%lf %lf",move_x_scale,move_y_scale);
                fclose(fp);
                */

            }
        break;
            
      }
    }   
        
        
     
}


/**
* This is ROS node to track the destination image
*/
int main(int argc, char **argv)
{
      move_x_scale = 1.0;
      move_y_scale = 1.0;
      move_path_buff.x = -1;
      move_path_buff.y = -1;
      
      updata_disp_set_config(1);
    /*    fp = fopen("/home/po/disp_GUI_set_conf.txt","r");
        fscanf(fp,"%lf %lf ",&move_x_scale,&move_y_scale);
        fclose(fp);
    */
    move_path.x = 0;
    move_path.y = 0;
  
     Map_image = Mat::zeros(200, 200, CV_8UC3);
    for(int height = 0; height< Map_image.rows;height++)
    {
        uchar *data = Map_image.ptr<uchar>(height);
        for(int width =0;width<Map_image.cols*3 ;width+=3)
        {
            data[width+0] =250;
            data[width+1] =251;
            data[width+2] =251;
            
        }
    }





   // std::string video_node = "usb_cam/image_raw";

    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;


    std::string video_node ; 
    int iii = 0;

    if(nh.getParam("/Graphical_User_Interface_1/video_node",video_node))
    {
      ROS_INFO("get %s",video_node.c_str());
    }
   else{
     video_node = "usb_cam/image_raw"; 
     ROS_INFO("video_node = usb_cam/image_raw");
   }
    if(nh.getParam("/Graphical_User_Interface_1/ime_w",iii))
    {
      ROS_INFO("get %i",iii);
    }
   else{
     ROS_ERROR("------ii");
   }



    ROS_INFO("get %s",video_node.c_str());
    ROS_INFO("-----------------");
   // ROS_INFO("get %s",video_node.c_str());
   //return(0);

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
  // image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
   image_transport::Subscriber sub = it.subscribe(video_node, 1, imageCallback);




 //   image_transport::Subscriber sub = it.subscribe("/camera_array/cam0/image_raw", 1, imageCallback);

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

void updata_disp_set_config(int w_r){

    //-long Map_origin_x,Map_origin_y,Map_end_x,Map_end_y;

    if(w_r == 1) //read
    {
       fp = fopen("/home/po/disp_GUI_set_conf.txt","r");
       fscanf(fp,"%lf %lf %ld %ld %ld %ld",&move_x_scale,&move_y_scale,&Map_origin_x,&Map_origin_y,&Map_end_x,&Map_end_y);
       fclose(fp);
    }
     if(w_r == 2) //write
    {
         fp = fopen("/home/po/disp_GUI_set_conf.txt","w");
         fprintf(fp,"%lf %lf %ld %ld %ld %ld",move_x_scale,move_y_scale,Map_origin_x,Map_origin_y,Map_end_x,Map_end_y);
         fclose(fp);

    }

}