// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// #include "exec/qr_robot_runner.h"
// #include "sim/aliengo_sim.h"
// #include "ros/control2gazebo_msg.h"
#include "quadruped/exec/qr_robot_runner.h"
#include "quadruped/robots/qr_robot_aliengo_sim.h"
#include "quadruped/ros/qr_control2gazebo_msg.h"
#include "quadruped/robots/qr_robot_sim.h"
#include "quadruped/gait/qr_walk_gait_generator.h"
#include "quadruped/gait/qr_gait.h"
#include "quadruped/gait/qr_openloop_gait_generator.h"

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_srvs/Empty.h>
#include <stdio.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>

using namespace std;
using namespace Quadruped;

bool startControllers(ros::NodeHandle &n, std::string serviceName, std::vector<std::string> &controllersToStart)
{

    ros::ServiceClient switchController = n.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switchControllerMsg;
    switchControllerMsg.request.start_controllers = controllersToStart;
    switchControllerMsg.request.strictness = switchControllerMsg.request.STRICT;
    ros::service::waitForService(serviceName, -1);
    switchController.call(switchControllerMsg);

    if (switchControllerMsg.response.ok){
        ROS_INFO_STREAM("Controller start correctly");
        return true;
    } else {
        ROS_ERROR_STREAM("Error occured trying to start controller");
        return false;
    }
}

bool stopControllers(ros::NodeHandle &n, std::string serviceName, std::vector<std::string> &controllers_to_stop)
{
    ros::ServiceClient switchController = n.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switchControllerMsg;
    switchControllerMsg.request.stop_controllers = controllers_to_stop;
    switchControllerMsg.request.strictness = switchControllerMsg.request.STRICT;
    switchControllerMsg.request.start_asap = false;
    ros::service::waitForService(serviceName, -1);
    switchController.call(switchControllerMsg);
    if (switchControllerMsg.response.ok){
        ROS_INFO_STREAM("Controller stop correctly");
        return true;
    } else {
        ROS_ERROR_STREAM("Error occured trying to stop controller");
        return false;
    }
}

// bool ResetRobot() {
//     int flag = system("rosservice call gazebo/delete_model '{model_name: aliengo_gazebo}'");
//     ROS_INFO("delete statu: %d", flag);    
//     int statu0 = system("rosrun gazebo_ros spawn_model -urdf -z 0.15 -model aliengo_gazebo -param robot_description -unpause");
//     ROS_INFO("spawn model statu: %d", statu0);
//     int statu1 = system("rosrun controller_manager spawner __ns:=/aliengo_gazebo joint_state_controller "\
//           "FL_hip_controller FL_thigh_controller FL_calf_controller "\
//           "FR_hip_controller FR_thigh_controller FR_calf_controller "\
//           "RL_hip_controller RL_thigh_controller RL_calf_controller "\
//           "RR_hip_controller RR_thigh_controller RR_calf_controller &");
//     ROS_INFO("controller statu: %d", statu1);
//     sleep(1);
//     return true;
//  }

bool ResetRobot(ros::ServiceClient &modelStateClient, ros::ServiceClient &jointStateClient)
{
    gazebo_msgs::ModelState modelState;
    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::SetModelConfiguration setjointstate;

    modelState.model_name = "lite3_gazebo";
    modelState.reference_frame = "world";

    geometry_msgs::Twist model_twist;
    geometry_msgs::Pose model_pose;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
    model_pose.position.x =0; // 4.5
    model_pose.position.y =0;
    model_pose.position.z =0.3; // 1.3, 0.3
    model_pose.orientation.w = 1;
    model_pose.orientation.x = 0;
    model_pose.orientation.y = 0;
    model_pose.orientation.z = 0;

    modelState.twist = model_twist;
    modelState.pose = model_pose;

    setmodelstate.request.model_state = modelState;
    setjointstate.request.model_name = "lite3_gazebo";
    setjointstate.request.urdf_param_name = "robot_description";
    setjointstate.request.joint_names = {"FR_HipX","FR_HipY", "FR_Knee",
                                         "FL_HipX","FL_HipY", "FL_Knee",
                                         "HR_HipX","HR_HipY", "HR_Knee",
                                         "HL_HipX","HL_HipY", "HL_Knee"};
    double hip_angle = 0.167136;
    double thigh_angle = -0.934969;
    double calf_angle = 2.54468;

    setjointstate.request.joint_positions = {-hip_angle,thigh_angle,calf_angle,
                                             hip_angle,thigh_angle,calf_angle,
                                             -hip_angle,thigh_angle,calf_angle,
                                             hip_angle,thigh_angle,calf_angle};

    ros::service::waitForService("/gazebo/set_model_state", -1);
    modelStateClient.call(setmodelstate);
    ros::service::waitForService("/gazebo/set_model_configuration", -1);
    if (jointStateClient.call(setjointstate))
    {
        ROS_INFO("BRILLIANT!!!");
        return true;
    } else
    {
        ROS_ERROR("Failed to set joints");
        return false;
    }
}

int main(int argc, char **argv)
{
    //std::string homeDir = GetHomeDir();
    std::string homeDir = ros::package::getPath("quadruped") + "/";
    std::string robotName = "lite3_sim";

    ros::init(argc, argv, "lite3_sim_as_a1");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    YAML::Node mainConfig = YAML::LoadFile(homeDir + "config/" + robotName + "/main.yaml");


    int twistMode = mainConfig["speed_update_mode"].as<int>();
    vector<float> linearVel = mainConfig["const_twist"]["linear"].as<vector<float >>();
    Vec3<float> desiredSpeed(0,0,0);
    float desiredTwistingSpeed = 0;
   

    desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();
    std::cout <<"speed  \n"<< desiredSpeed << desiredTwistingSpeed << std::endl;


 
    

    std::vector<std::string> controllerList = {"joint_state_controller", "FL_hip_controller", "FL_thigh_controller",
                                               "FL_calf_controller", "FR_hip_controller", "FR_thigh_controller",
                                               "FR_calf_controller", "RL_hip_controller", "RL_thigh_controller",
                                               "RL_calf_controller", "RR_hip_controller", "RR_thigh_controller",
                                               "RR_calf_controller"};

    // ros::init(argc, argv, "aliengo_sim");
   

    ros::ServiceClient modelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient jointStateClient = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    bool flag = ResetRobot(modelStateClient, jointStateClient);
    ROS_INFO("Reset the Robot pose");
    // bool flag = ResetRobot();
    // ROS_INFO("Reset the Robot pose");

    

    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    

    ROS_INFO("ROS node init finished");

    qrRobot *quadruped = new qrRobotSim(nh, privateNh, "lite3", homeDir);
    quadruped->timeStep = 1.0 / 1000.0; //userParameters.controlFrequency;
    
    quadruped->ReceiveObservation();

    // Robot *quadruped = new AliengoSim(nh, privateNh, homeDir + "config/aliengo_sim/aliengo_sim.yaml");
 
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;
    Action::StandUp(quadruped, 4.f, 5.f, 0.001);
    // Action::SitDown(quadruped, 3, 0.001);
    std::cout << "control mode:\n" << quadruped->controlParams["mode"]<< std::endl;
    std::cout << "walk code" << LocomotionMode::WALK_LOCOMOTION << std::endl;
    // if (quadruped->controlParams["mode"] == LocomotionMode::WALK_LOCOMOTION) {
    // qrGaitGenerator *gaitGenerator = new qrWalkGaitGenerator(quadruped, homeDir + "config/" + quadruped->robotName
    //                                                     + "/openloop_gait_generator.yaml");
    // } else {
    qrGaitGenerator *gaitGenerator = new qrOpenLoopGaitGenerator(quadruped, homeDir + "config/" + quadruped->robotName
                                                         + "/openloop_gait_generator.yaml");
    // }                                                                 
    std::cout << "init gaitGenerator finish\n" << std::endl;
    std::cout << quadruped->controlParams["mode"] << "\n" << std::endl;

    qrDesiredStateCommand *desiredStateCommand = new qrDesiredStateCommand(nh, quadruped);
    desiredStateCommand->vDesInBodyFrame = desiredSpeed;
    desiredStateCommand->wDesInBodyFrame << 0,0, desiredTwistingSpeed;
    
    qrUserParameters *userParameters = new qrUserParameters(homeDir+ "config/user_parameters.yaml");
    Eigen::MatrixXf::Map(&userParameters->desiredSpeed[0], 3, 1) = desiredSpeed;

    userParameters->desiredTwistingSpeed = desiredTwistingSpeed;
    std::cout << "update user params \n"<< userParameters->desiredSpeed << std::endl;
    std::cout << "use wbc: " << userParameters->useWBC << std::endl;

    qrStateEstimatorContainer *stateEstimators = new qrStateEstimatorContainer(quadruped, gaitGenerator, userParameters, 
                                                        "config/" + quadruped->robotName + "/terrain.yaml", 
                                                        homeDir); 



    qrLocomotionController *locomotionController = SetUpController(quadruped, 
                                                                    gaitGenerator, 
                                                                    desiredStateCommand, 
                                                                    stateEstimators, 
                                                                    userParameters, 
                                                                    homeDir);
    ROS_INFO("LocomotionController Init Finished");
    locomotionController->Reset();
    ROS_INFO("LocomotionController Reset Finished");

    // ros module init
    // RobotOdometryEstimator *legOdom = new RobotOdometryEstimator(quadruped, locomotionController, nh);
    // CmdVelReceiver *cmdVelReceiver = new CmdVelReceiver(nh, privateNh);
    // SLAMPoseReceiver *slamPoseReceiver = new SLAMPoseReceiver(nh, privateNh);
    // SwitchModeReceiver *switchModeReceiver = new SwitchModeReceiver(nh, privateNh);
    // ros::ServiceClient baseStateClient = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

    // qrController2GazeboMsg *controller2gazeboMsg = new qrController2GazeboMsg(quadruped, locomotionController, nh);
    // ROS_INFO("ROS Modules Init Finished");

    ROS_INFO("TimeSinceReset: %f", quadruped->GetTimeSinceReset());

    UpdateControllerParams(locomotionController, {0., 0., 0.}, 0.);

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    ROS_INFO("start control loop....");

    int switchMode;
    std::cout << ros::ok();
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();        
        // switchMode = switchModeReceiver->GetSwitchMode();

        // if (twistMode == TwistMode::ROS) {
        //     desiredSpeed = cmdVelReceiver->GetLinearVelocity();
        //     desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity();
        // }

        // if (switchMode != 2 && quadruped->controlParams["mode"] != switchMode) {
        //     ROS_INFO_STREAM("switch mode from " << quadruped->controlParams["mode"] << " to " << switchMode);
        //     SwitchMode<A1Sim>(quadruped, locomotionController, desiredSpeed, desiredTwistingSpeed, switchMode, startTimeWall);
        // }
        UpdateControllerParams(locomotionController,
                                   desiredSpeed,
                                   desiredTwistingSpeed);
        locomotionController->Update();
        auto[hybridAction, qpSol] = locomotionController->GetAction();
        quadruped->Step(qrMotorCommand::convertToMatix(hybridAction), HYBRID_MODE);

        // // ros
        // legOdom->PublishOdometry();

        // currentTime = quadruped->GetTimeSinceReset();
        // if (abs(quadruped->baseRollPitchYaw[0]) > 0.5f || abs(quadruped->baseRollPitchYaw[1]) > 0.5f) {
        //     ROS_ERROR("The dog is going down, main function exit.");
        //     break;
        // }
        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
    }
    ROS_INFO("Time is up, end now.");
    int value = system("ps -ef | grep controller_manager| awk '{print $2}'| xargs -i kill -9 {} > /dev/null 2>&1");
    ros::shutdown();
    return 0;
}
