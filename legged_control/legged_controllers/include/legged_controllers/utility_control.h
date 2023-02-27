
#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <legged_controllers/legged_controller.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <legged_interface/legged_interface.h>
namespace ocs2 {
namespace legged_robot {
class Utilitypolicy : public SolverSynchronizedModule {
 public:
  Utilitypolicy(legged::LeggedInterface& legged_interface,
                std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                std::map<std::string, ModeSequenceTemplate> gaitMapconst ,
                vector_t& optcurrent,
               std::vector<legged::ContactSensorHandle>& contact_sensor_handles,
                const std::string& robotName);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

private:
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  legged::LeggedInterface&  legged_interface_;
  vector_t& optcurrent_;
  std::map<std::string, ModeSequenceTemplate> gaitMapconst_;
  std::vector<legged::ContactSensorHandle>& contact_sensor_handles_;
};

}
}
