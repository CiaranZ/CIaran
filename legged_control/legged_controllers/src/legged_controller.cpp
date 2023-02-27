//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/legged_controller.h"

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

#include <legged_estimation/from_topice_estimate.h>
#include <legged_estimation/linear_kalman_filter.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include "legged_controllers/utility_control.h"

namespace legged
{
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Initialize OCS2
  std::string task_file, urdf_file, reference_file,gaitCommandFile;
  controller_nh.getParam("/task_file", task_file);
  controller_nh.getParam("/urdf_file", urdf_file);
  controller_nh.getParam("/reference_file", reference_file);
  controller_nh.getParam("/gaitCommandFile", gaitCommandFile);
  bool verbose;
  loadData::loadStdVector(gaitCommandFile, "list", gaitList_, verbose);
  loadData::loadCppDataType(task_file, "legged_robot_interface.verbose", verbose);
  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitCommandFile, gaitName, verbose)});
  }
  setupLeggedInterface(task_file, urdf_file, reference_file, verbose);        //初始化legged_interface,初始化优化问题
  setupMpc();
  setupMrt();

  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
  PinocchioEndEffectorKinematics ee_kinematics(legged_interface_->getPinocchioInterface(), pinocchio_mapping,
                                               legged_interface_->modelSettings().contactNames3DoF);
  visualizer_ = std::make_shared<LeggedRobotVisualizer>(legged_interface_->getPinocchioInterface(),
                                                        legged_interface_->getCentroidalModelInfo(), ee_kinematics, nh);


  // Hardware interface
  HybridJointInterface* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{ "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
                                        "LF_foot_fixed","LH_foot_fixed","RF_foot_fixed","RH_foot_fixed"};

  for (const auto& joint_name : joint_names)
    hybrid_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));
  ContactSensorInterface* contact_interface = robot_hw->get<ContactSensorInterface>();

  for (auto& name : legged_interface_->modelSettings().contactNames3DoF)
    contact_handles.push_back(contact_interface->getHandle(name));                  //   这里面和硬件对接，相当于读编码器和01的接触传感器


  // State estimation
  setupStateEstimate(*legged_interface_, hybrid_joint_handles_,contact_handles,
                     robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu"));
  // Whole body control
  wbc_ = std::make_shared<Wbc>(task_file, *legged_interface_, ee_kinematics, verbose);
  // Safety Checker
  safety_checker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());
  ROS_INFO_STREAM("Start init10.");
  return true;
}

void LeggedController::starting(const ros::Time& time)
{
  ROS_INFO_STREAM("Start Srarting.");
  // Initial state
  current_observation_.mode = ModeNumber::STANCE;
  current_observation_.state =
      rbd_conversions_->computeCentroidalStateFromRbdModel(state_estimate_->update(time, ros::Duration(0.002)));
  current_observation_.input.setZero(legged_interface_->getCentroidalModelInfo().inputDim);
  TargetTrajectories target_trajectories({ current_observation_.time }, { current_observation_.state },
                                         { current_observation_.input });
  // std::cerr << "current_observation_.state " << current_observation_.state<< std::endl;

  // std::cerr << "current_observation_.input. " << current_observation_.input<< std::endl;

  // Set the first observation and command and wait for optimization to finish
  mpc_mrt_interface_->setCurrentObservation(current_observation_);                      //互斥锁，锁住当前状态
  mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  optimized_state = current_observation_.state;
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpc_mrt_interface_->initialPolicyReceived() && ros::ok())         //前一项在上面初始化的地方就是false了，所以没问题
  { 
    mpc_mrt_interface_->advanceMpc();
    ROS_INFO_STREAM("finish advanceMpc ");
    ros::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpc_running_ = true;
}


//假设现在是站立情况，然后可以前后移动
void LeggedController::update(const ros::Time& time, const ros::Duration& period)
{
  // ROS_INFO_STREAM("Start UPdate.");
  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
  PinocchioEndEffectorKinematics ee_kinematics(legged_interface_->getPinocchioInterface(), pinocchio_mapping,
                                               legged_interface_->modelSettings().contactNames3DoF);
  ee_kinematics.setPinocchioInterface(legged_interface_->getPinocchioInterface());

  const auto& model = legged_interface_->getPinocchioInterface().getModel();
  auto& data = legged_interface_->getPinocchioInterface().getData();

  // State Estimate
  current_observation_.time += period.toSec();

  vector_t measured_rbd_state = state_estimate_->update(time, period);      //
  scalar_t yaw_last = current_observation_.state(9);                    // 前面6个是转矩，7，8是pitch,row,9是YAW
  current_observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state);        //state实际是16+6+6，但是需要把他简化成12+6+6
  current_observation_.state(9) = yaw_last + angles::shortest_angular_distance(yaw_last, current_observation_.state(9));
  current_observation_.mode = state_estimate_->getMode();
  
  // Update the current state of the system
  mpc_mrt_interface_->setCurrentObservation(current_observation_);
  // std::cerr << "current_observation_.state " << mpc_mrt_interface_->getReferenceManager().getTargetTrajectories().stateTrajectory[1]<<std::endl;
  // std::cerr << "current_observation_.state "<<current_observation_.state<<std::endl;
  // // Load the latest MPC policy
  mpc_mrt_interface_->updatePolicy();

  // Evaluate the current policy

  size_t planned_mode;  // The mode that is active at the time the policy is evaluated at.
  mpc_mrt_interface_->evaluatePolicy(current_observation_.time, current_observation_.state, optimized_state,
                                     optimized_input, planned_mode);//MPC一直在别的线程跑，所以这里直接更新policy就行

  // Whole body control
  current_observation_.input = optimized_input;
  vector_t x = wbc_->update(optimized_state, optimized_input, measured_rbd_state, planned_mode);

  vector_t torque = x.tail(12);

  vector_t pos_des = centroidal_model::getJointAngles(optimized_state, legged_interface_->getCentroidalModelInfo());
  vector_t vel_des = centroidal_model::getJointVelocities(optimized_input, legged_interface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safety_checker_->check(current_observation_, optimized_state, optimized_input))
  {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }
  pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(current_observation_.state, legged_interface_->getCentroidalModelInfo()));
  pinocchio::updateFramePlacements(model, data);
  auto tempposition = ee_kinematics.getPosition(current_observation_.state);  
  auto state_ref = mpc_mrt_interface_->getReferenceManager().getTargetTrajectories().stateTrajectory.back();
  pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(optimized_state, legged_interface_->getCentroidalModelInfo()));
  pinocchio::updateFramePlacements(model, data);
  auto feetPositions = ee_kinematics.getPosition(optimized_state);

  for (size_t j = 0; j < legged_interface_->getCentroidalModelInfo().actuatedDofNum; ++j)
    hybrid_joint_handles_[j].setCommand(pos_des(j), vel_des(j), 5, 3, torque(j));

  std::vector<bool> is_contact;
  for (size_t j = 0; j < 4; j++)
    is_contact.push_back(  contact_handles[j].isContact());


  is_contact[0]?hybrid_joint_handles_[12+0].setCommand(0, 0, 0, 0, 5*(feetPositions[0][0]- tempposition[0][0])):hybrid_joint_handles_[12+0].setCommand(0, 0, 0, 0, 0);//l
  is_contact[2]?hybrid_joint_handles_[12+1].setCommand(0, 0, 0, 0, 5*(feetPositions[2][0]- tempposition[2][0])):hybrid_joint_handles_[12+1].setCommand(0, 0, 0, 0, 0);//l
  is_contact[1]?hybrid_joint_handles_[12+2].setCommand(0, 0, 0, 0, 5*(feetPositions[1][0]- tempposition[1][0])):hybrid_joint_handles_[12+2].setCommand(0, 0, 0, 0, 0);//r
  is_contact[3]?hybrid_joint_handles_[12+3].setCommand(0, 0, 0, 0, 5*(feetPositions[3][0]- tempposition[3][0])):hybrid_joint_handles_[12+3].setCommand(0, 0, 0, 0, 0);//r


  // Visualization
  visualizer_->update(current_observation_, mpc_mrt_interface_->getPolicy(), mpc_mrt_interface_->getCommand());

  // Publish the observation. Only needed for the command interface
  observation_publisher_.publish(ros_msg_conversions::createObservationMsg(current_observation_));
}
LeggedController::~LeggedController()
{
  controller_running_ = false;
  if (mpc_thread_.joinable())
    mpc_thread_.join();
}

void LeggedController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                            const std::string& reference_file, bool verbose)
{
  legged_interface_ = std::make_shared<LeggedInterface>(task_file, urdf_file, reference_file, verbose);
  legged_interface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void LeggedController::setupMpc()
{
  //  mpc_ = std::make_shared<GaussNewtonDDP_MPC>(legged_interface_->mpcSettings(), legged_interface_->ddpSettings(),
  //                                              legged_interface_->getRollout(),
  //                                              legged_interface_->getOptimalControlProblem(),
  //                                              legged_interface_->getInitializer());
  mpc_ = std::make_shared<MultipleShootingMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                               legged_interface_->getOptimalControlProblem(),
                                               legged_interface_->getInitializer());
  rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                     legged_interface_->getCentroidalModelInfo());

  const std::string robot_name = "legged_robot";
  ros::NodeHandle nh;
  auto utilitypolicy = std::make_shared<Utilitypolicy>(*legged_interface_,
                                                       legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
                                                       gaitMap_,
                                                       optimized_state,
                                                       contact_handles,
                                                       robot_name);
  // Gait receiverUtilitypolicy
  auto gait_receiver_ptr = std::make_shared<GaitReceiver>(
      nh, legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robot_name);
  // ROS ReferenceManager
  auto ros_reference_manager_ptr =
      std::make_shared<RosReferenceManager>(robot_name, legged_interface_->getReferenceManagerPtr());
  ros_reference_manager_ptr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gait_receiver_ptr);
  mpc_->getSolverPtr()->addSynchronizedModule(utilitypolicy);
  mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_ptr);
  observation_publisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robot_name + "_mpc_observation", 1);
}

void LeggedController::setupMpcutility()
{

  const std::string robot_name = "legged_robot";
  auto gait_receiver_ptr = std::make_shared<GaitReceiver>(
  *legged_interface_, *mpc_mrt_interface_,current_observation_.state,legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robot_name);
  mpc_->getSolverPtr()->addSynchronizedModule(gait_receiver_ptr);

}

void LeggedController::setupMrt()
{
  mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());

  controller_running_ = true;
  mpc_thread_ = std::thread([&]() {
    while (controller_running_)
    {
      try
      {
        executeAndSleep(
            [&]() {
              if (mpc_running_)
                mpc_mrt_interface_->advanceMpc();
            },
            legged_interface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controller_running_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
      }
    }
  });
  setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
}

void LeggedController::setupStateEstimate(LeggedInterface& legged_interface,
                                          const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                          const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                          const hardware_interface::ImuSensorHandle& imu_sensor_handle)
{
  state_estimate_ = std::make_shared<KalmanFilterEstimate>(*legged_interface_, hybrid_joint_handles_,
                                                           contact_sensor_handles, imu_sensor_handle);
  current_observation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(LeggedInterface& legged_interface,
                                                 const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                                 const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                                 const hardware_interface::ImuSensorHandle& imu_sensor_handle)
{
  state_estimate_ = std::make_shared<FromTopicStateEstimate>(*legged_interface_, hybrid_joint_handles_,
                                                             contact_sensor_handles, imu_sensor_handle);
}









}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
