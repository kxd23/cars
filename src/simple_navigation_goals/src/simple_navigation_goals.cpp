#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const int key_points = 3;
ros::Subscriber sub;
int id;
double rad[key_points],x_rad,y_rad,x[key_points]={1,3,5},y[key_points]={0,0,0};
double x_real[key_points],y_real[key_points];
bool flag[key_points];

// 回调函数s
void radiation_sub(const std_msgs::Float64MultiArray msg_s){
    // 通过msg_p来获取并操作订阅到的数据
    // ROS_INFO("meow %f\n",msg_s.data);
    if(!flag[id]){
        // flag[id]=true;
        // rad[id]=msg_s.data[0];
        // x_real[id]=msg_s.data[1];
        // y_real[id]=msg_s.data[2];
        rad[id]=msg_s.data.at(0);
        x_real[id]=msg_s.data.at(1);
        y_real[id]=msg_s.data.at(2);
    }
}

double inv_dist(double x1,double y1,double x2,double y2){
    return 20/sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void get_rad_xy(){
    // for(int i=0;i<key_points;++i){
    //     ROS_INFO("wang %d %lf %lf %lf %lf\n",i,rad[i],x_real[i],y_real[i],inv_dist(x_real[i],y_real[i],0,-5.6));
    // }
    double ans=100;
    for(double i=0;i<6;i+=0.1){
        for(double j=-6;j<0;j+=0.1){
            double t1=(rad[2]-rad[0])/(rad[1]-rad[0]);
            double t2=(inv_dist(x_real[2],y_real[2],i,j)-inv_dist(x_real[0],y_real[0],i,j))/(inv_dist(x_real[1],y_real[1],i,j)-inv_dist(x_real[0],y_real[0],i,j));
            if(ans>fabs(t1-t2)){
                ans=fabs(t1-t2);
                x_rad=i;
                y_rad=j;
            }
            // ROS_INFO("get_rad %lf %lf %lf\n",i,j,fabs(t1-t2));
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");

    ros::NodeHandle nh;
    sub=nh.subscribe<std_msgs::Float64MultiArray>("radiation",10,radiation_sub);

    MoveBaseClient ac("move_base", true);

    move_base_msgs::MoveBaseGoal goal;


    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    for(int i=0;i<key_points;++i){
        // 到达第i个点并收集辐射信号
        ROS_INFO("%d?",i);
        goal.target_pose.pose.position.x = x[i];
        goal.target_pose.pose.position.y = y[i];
        ac.sendGoal(goal);
        ac.waitForResult();
        id=i;
        ros::spinOnce();
        ROS_INFO("%d!",i);
    }

    get_rad_xy();

    goal.target_pose.pose.position.x = x_rad;
    goal.target_pose.pose.position.y = y_rad;
    ROS_INFO("meow %lf %lf\n",x_rad,y_rad);
    ac.sendGoal(goal);
    ac.waitForResult();
    return 0;
}
