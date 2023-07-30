#include <MA_dt_quadrotor/ControlEval.h>
#include "dynamic_model_quad.cpp"

class DigitalTwin
{

protected:
    ros::NodeHandle nh_;
    boost::thread *spin_thread;
    bool stop_thread=false;
    std::string STATE_TOPIC;


    mavros_msgs::State  current_state;
    geometry_msgs::Pose current_pose;
    dt_test_pkg::Wind   current_wind;

    ros::Subscriber pose_sub;
    ros::Subscriber wind_sub;
    ros::Subscriber state_sub;

    ros::Publisher control_pub;

    ros::Publisher local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    

    ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    QuadDynamicModel _dynamic_model_quad;
            


    
public:
    void spinThread()
    {
        while(!stop_thread)
        {
            ros::spinOnce();
        }
    }

    void stateCallBack(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    DigitalTwin() 
    {
        nh_.param("ROSTOPIC_STATE_RAW", STATE_TOPIC, STATE_TOPIC);
        
        // update the info of environmental and digital model
        state_sub = nh_.subscribe<mavros_msgs::State>(STATE_TOPIC, 10, &DigitalTwin::stateCallBack, this);
        pose_sub = nh_.subscribe("/physical_entity/local_position/pose",10, &DigitalTwin::poseCallBack,this);
        wind_sub = nh_.subscribe("/environment/wind",10, &DigitalTwin::windCallBack,this);

        // check connection
        ros::Rate rate(20.0);
        while(ros::ok() && !current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

        stop_thread = false;
        spin_thread = new boost::thread(&DigitalTwin::spinThread,this);

    }

    ~DigitalTwin()
    {
        ROS_WARN_STREAM("Destroy the object.");
        stop_thread=true;
        sleep(1);
        // spin_thread->join();
    }

    //------------------------ CALLBACK ------------------------//

    void poseCallBack(const geometry_msgs::Pose::ConstPtr& msg ){
        current_pose = *msg;
        // ROS_INFO_STREAM("[poseCallBack] Get pose data.");

    }

    void windCallBack(const dt_test_pkg::Wind::ConstPtr& msg){
        current_wind = *msg;
        // ROS_INFO_STREAM("[windCallBack] Get wind data.");

    }

    //------------------------ main loop ------------------------//

    void main_loop(){
        ROS_INFO_STREAM("[main_loop] Start main task.");
        int display_count = 0;
        

        ros::Rate rate(20.0); // setpoint frequency must be faster than 20HZ

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 6;
        pose.pose.position.y = 0;
        pose.pose.position.z = 1;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;


        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();

        while(ros::ok()){
          
            // update dynamic model
            if( display_count % 10 == 0){
                _dynamic_model_quad.updateModelParameters();
                _dynamic_model_quad.updateForceandMoment();
                _dynamic_model_quad.newtonEulerMotionEquation();
                _dynamic_model_quad.printInfo();
                
              
            }
            display_count++;
            
            ros::spinOnce();
            rate.sleep();

        }

    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "twin_node");

    DigitalTwin digital_drone;
    ROS_INFO_STREAM("[MAIN] Object of DigitalTwin i[MAIN] Object of DigitalTwin is built. The digital twin of drone starts updating data."); 

    digital_drone.main_loop();
  
    return 0;
}
