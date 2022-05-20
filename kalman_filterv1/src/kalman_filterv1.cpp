#include "ros/ros.h"
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>
#include <vector>
#include <fstream>
#include <math.h>


 std::vector<int> xcord_array;
 std::vector<int> ycord_array;
 std::vector<unsigned long> t_array;
 std::vector<std::vector<int>> xcord_arrays;
 std::vector<std::vector<int>> ycord_arrays;
 int count = 0;
 std::ofstream myFile;


void xCallback(const dvs_msgs::EventArray::ConstPtr &msg){
    
    std::vector<dvs_msgs::Event> cord = msg->events;
    int xcord = cord[0].x;
    int ycord = cord[0].y;
    unsigned long secs = cord[0].ts.sec; 
    unsigned long nsecs = cord[0].ts.nsec;

    //myFile << secs << "," << nsecs << "\n";
    

    int siz = cord.size();

    // int xcord_e = cord[size-1].x;
    // int ycord_e = cord[size-1].y;

    for (int e = 0; e < siz; e++)
    {
        xcord_array.push_back(cord[e].x);
        ycord_array.push_back(cord[e].y);
        t_array.push_back(cord[e].ts.nsec*10^-9+cord[e].ts.sec);
    }
    myFile<<siz<<"\n";
    for(int i = 1; i<xcord_array.size();i++)
    {
        myFile << xcord_array[i] << ","; 
    }
    myFile<<"\n";
    for(int i = 1; i<xcord_array.size();i++)
    {
        myFile << ycord_array[i] << ","; 
    }
    myFile<<"\n";
    for(int i = 1; i<xcord_array.size();i++)
    {
        myFile << t_array[i] << ","; 
    }
    myFile<<"\n";
    
    
    // xcord_arrays.push_back(xcord_array);
    // ycord_arrays.push_back(ycord_array);

      int xsize = xcord_array.size();
      int ysize = ycord_array.size();

    // // //feature_msg.events.push_back(e);

      
    // // //xcor = msg->events;
    // // //float x_cord = xmsg.x;
    // // ROS_INFO("ping4 \n");
    // // ROS_INFO("First x cord %i",xcord);
    // // ROS_INFO("First y cord %i",ycord);
     count++;
     ROS_INFO("Message num %i",count);
     ROS_INFO("size of array? %i",siz);
    // // ROS_INFO("Last x cord %i",xcord_e);
    // // ROS_INFO("Last y cord %i \n",ycord_e);

     ROS_INFO("size of array? %i",xsize);
    // // ROS_INFO("First x cord %i",xcord_array[0]);
    // // ROS_INFO("Last x cord %i",xcord_array[xsize-1]);

     ROS_INFO("size of array? %i",ysize);
    // // ROS_INFO("First y cord %i",ycord_array[0]);
    // // ROS_INFO("Last y cord %i \n",ycord_array[ysize-1]);

     xcord_array.clear();
     ycord_array.clear();

    //ros::shutdown();
    return;
    }

int main(int argc, char** argv){
    //myFile.open("~/Documents/times.csv");
    myFile.open("cordst.csv", std::ios::out | std::ios::binary);
    ROS_INFO("ping");
    ros::init(argc, argv, "kalman_filterv1");
    ROS_INFO("ping2");
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/feature_events",0,xCallback);
    ROS_INFO("ping3");
            
    //ROS_INFO("pol: %b",pol);
    //ROS_INFO("%d",pol);
    ros::Rate r(1);
    
    ros::spin(); 
    ROS_INFO("ping4");
    myFile.close();        
    ros::shutdown();
    
    
    
     
    //ros::spin();
    return 0;
}