#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "legged_controllers/utility_control.h"
#include <algorithm>

namespace ocs2 {
namespace legged_robot {

Utilitypolicy::Utilitypolicy(legged::LeggedInterface& legged_interface, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                          std::map<std::string, ModeSequenceTemplate> gaitMapconst, vector_t& optcurrent, std::vector<legged::ContactSensorHandle>& contact_sensor_handles,
                             const std::string& robotName)
: legged_interface_(legged_interface),gaitMapconst_(gaitMapconst),gaitSchedulePtr_(std::move(gaitSchedulePtr)),optcurrent_(optcurrent),contact_sensor_handles_(contact_sensor_handles)
{

}

void Utilitypolicy::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                 const ReferenceManagerInterface& referenceManager)
{

  const auto timeHorizon = finalTime - initTime;
  static auto gaitCommand="stance";
  static auto gaitCommandold="none";
  std::vector<std::pair<size_t, double>> vtMap;

  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_.getCentroidalModelInfo());
  PinocchioEndEffectorKinematics ee_kinematics(legged_interface_.getPinocchioInterface(), pinocchio_mapping,
                                               legged_interface_.modelSettings().contactNames3DoF);
  const auto& model = legged_interface_.getPinocchioInterface().getModel();
  auto& data = legged_interface_.getPinocchioInterface().getData();

  ee_kinematics.setPinocchioInterface(legged_interface_.getPinocchioInterface());

  auto currenteepos = ee_kinematics.getPosition(currentState);  
  static std::vector<vector3_t> currenteepos_contact;  
  auto state_final = referenceManager.getTargetTrajectories().stateTrajectory.back();
  auto state_start = referenceManager.getTargetTrajectories().stateTrajectory.front();
  static vector_t  state_start_last = state_start;
  pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state_start, legged_interface_.getCentroidalModelInfo()));
  pinocchio::updateFramePlacements(model, data);
  auto feetPositions_start = ee_kinematics.getPosition(state_start);
  pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state_final, legged_interface_.getCentroidalModelInfo()));
  pinocchio::updateFramePlacements(model, data);
  auto feetPositions_opt = ee_kinematics.getPosition(state_final);

  auto composition = currentState.segment<3>(6);
  auto composition_ref = state_final.segment<3>(6);
  auto composition_start = state_start.segment<3>(6);
    std::cerr << "debug3" << "\n";
  if(currenteepos_contact.empty())
  {
    currenteepos_contact = currenteepos;
  }

  // if(state_start_last.empty())
  // {
  //   state_start_last   =  state_start;
  // }
  for (size_t i = 0; i < 4; i++)
  {
    if(contact_sensor_handles_[i].isContact())
      currenteepos_contact[i] = currenteepos[i];
  }

  std::vector<vector3_t>  com2pfeet(4);
  for (size_t i = 0; i < 4; i++)
  {
    com2pfeet[i]= currenteepos_contact[i] - composition +composition_ref-composition;    // 每时每刻的
    // std::cerr << "leg com2pfeet No."<<i<<":"<< com2pfeet[i]  << "\n";
  }
    std::cerr << "debug2" << "\n";
  static std::vector<vector3_t>  feet_ref;          //每次收到指令的时候
  if (feet_ref.empty())
  {

    feet_ref = com2pfeet;

  }
  std::cerr << "debug1" << "\n";
  if (state_start_last != state_start)
  {
    for (size_t i = 0; i < 4; i++)
    {
      feet_ref[i]= currenteepos_contact[i] - composition;
      vector3_t refvector= composition_ref-composition;  
      vector3_t unitwheelvector ;
      unitwheelvector(0) = cos(currentState(9));  //  这个不对
      unitwheelvector(1) = sin(currentState(9));
      auto pointby = unitwheelvector.dot(refvector);

      feet_ref[i][0] +=  pointby;
      // feet_ref[i][1] += pointby * sin(currentState(9));
      // feet_ref[i][2] = 0;
      std::cerr << "leg cos No."<<i<<": \n"<< cos(currentState(9)) << "\n";
      std::cerr << "leg sin No."<<i<<": \n"<< sin(currentState(9)) << "\n";
      std::cerr << "leg currenteepos_contact No."<<i<<":\n"<< currenteepos_contact[i] << "\n";
      std::cerr << "leg refvector No."<<i<<":\n"<< refvector << "\n";
      std::cerr << "leg unitwheelvector No."<<i<<":\n"<< unitwheelvector << "\n";
      std::cerr << "leg pointby No."<<i<<":\n"<< pointby << "\n";
      std::cerr << "leg feet_ref No."<<i<<":\n"<< feet_ref[i]  << "\n";
      std::cerr << "leg com2pfeet No."<<i<<":\n"<< com2pfeet[i]  << "\n";
    }
    state_start_last = state_start; 
  }
  std::vector<vector3_t>  beforemapping(4);
  for (size_t i = 0; i < 4; i++)
  {
    beforemapping[i] =com2pfeet[i]-feet_ref[i];
    // std::cerr << "leg beforemapping No."<<i<<":"<< beforemapping[i]  << "\n";
  }
  float lemada1 = 100;
  float lemada2 = 0.2;
  vector_t leguitility(4);
  for (size_t i = 0; i < 4; i++)
  {
    vector3_t unitwheelvector ;
    vector3_t unitwheelvectorv;
    unitwheelvector(0) = cos(currentState(9));
    unitwheelvector(1) = sin(currentState(9));
    unitwheelvectorv(0)= sin(currentState(9));
    unitwheelvectorv(1)= cos(currentState(9));
    leguitility[i] = 1.0-sqrt((((unitwheelvector.dot(beforemapping[i]))/lemada1)*((unitwheelvector.dot(beforemapping[i]))/lemada1))+(((unitwheelvectorv.dot(beforemapping[i]))/lemada2)*((unitwheelvectorv.dot(beforemapping[i]))/lemada2)));
    std::cerr << "leg lemada1 No."<<i<<":"<< ((unitwheelvector.dot(beforemapping[i]))/lemada1)  << "\n";
    std::cerr << "leg lemada2 No."<<i<<":"<< ((unitwheelvectorv.dot(beforemapping[i]))/lemada2)  << "\n";
    std::cerr << "leg uitility No."<<i<<":"<< leguitility[i]  << "\n";
  }
  
  static std::map<size_t, double> gaitCommandstack; 
  gaitCommandstack.clear();

  for (size_t i = 0; i < 4; i++)
  {
    if(leguitility[i]<0.8)
    {
      gaitCommandstack.insert({i,leguitility[i]});
      // std::cerr << "leg uitility No."<<i<<":"<< gaitCommandstack[i] <<"\n";
    }
  }

// //lflhrfrh
  if(!gaitCommandstack.empty())
  {
    vtMap.clear();
    for (auto it = gaitCommandstack.begin(); it != gaitCommandstack.end(); it++)
    {
        vtMap.push_back(std::make_pair(it->first, it->second));
        // std::cerr << "leg vtMap No."<<it->first<<":"<< it->second <<"\n";
    }

    // for (size_t i = 0; i < vtMap.size(); i++)
    // {
    //   std::cerr << "leg vtMap No."<<vtMap[i].first<<":"<< vtMap[i].second <<"\n";
    // }
    sort(vtMap.begin(), vtMap.end(), 
        [](const std::pair<size_t, double> &x, const std::pair<size_t, double> &y) -> size_t {
        return x.second < y.second;
    });

    static size_t filter_cnt = 0;
    static size_t old_feet = 0;
    //vtmap 按照value排列
    if(vtMap.size() == 1)
    {
      switch (vtMap[0].first)
      {
        case 0:   //lf
        {
          if(!contact_sensor_handles_[1].isContact() || !contact_sensor_handles_[2].isContact())
            gaitCommand = "lf_recover";
        }
          break;
        case 1://rf
        {
          if(!contact_sensor_handles_[0].isContact() || !contact_sensor_handles_[3].isContact())
            gaitCommand = "rf_recover";

        }

          break;
        case 2://lh
        {
          if(!contact_sensor_handles_[1].isContact() || !contact_sensor_handles_[3].isContact())
            gaitCommand = "lh_recover";

        }

          break;
        case 3://rh
        {
          if(!contact_sensor_handles_[1].isContact() || !contact_sensor_handles_[2].isContact())
            gaitCommand = "rh_recover";

        }
          break;
        default:
          break;
      }
    }
  
    else if(vtMap.size() == 4)
    {
      switch (vtMap[0].first)
      {
        case 0: 
        case 3:
        {
          if(!contact_sensor_handles_[1].isContact() || !contact_sensor_handles_[2].isContact())
            gaitCommand = "lf_rh_recover";
        }

          break;
        case 1: 
        case 2:
        {
          if(!contact_sensor_handles_[0].isContact() || !contact_sensor_handles_[3].isContact())
            gaitCommand = "lh_rf_recover";
        }
          break;

        default:
          break;
      }
    }
    else
    {
      switch (vtMap[0].first)
      {
        case 0://LF
          vtMap[1].first == 3 ?gaitCommand = "lf_rh_recover":gaitCommand = "lf_recover";
          break;
        case 1://rf
          vtMap[1].first == 2 ?gaitCommand = "lh_rf_recover":gaitCommand = "rf_recover";
          break;
        case 2:
          vtMap[1].first == 1 ?gaitCommand = "lf_rh_recover":gaitCommand = "lh_recover";
          break;
        case 3:
          vtMap[1].first == 0 ?gaitCommand = "lh_rf_recover":gaitCommand = "rh_recover";
          break;
        default:
          break;
      }
    }

  }
  else
  {
    gaitCommand = "stance";
  }

  if(gaitCommandold == gaitCommand)
  {
    std::cerr <<  "gaitCommand"<<gaitCommand<<"\n";;
  }
  else
  {
    std::cerr <<  "gaitCommand"<<gaitCommand<<"\n";
    gaitSchedulePtr_->insertModeSequenceTemplate(gaitMapconst_.at(gaitCommand),finalTime, timeHorizon);
  }
  gaitCommandold = gaitCommand;
  // if(leguitility[3]<0.5  &&  gaitCommand !="rh_recover")
  // {
  //   gaitCommand ="rh_recover";
  //   std::cerr << "leg uitility" << leguitility[3] << "gaitCommand"<<gaitCommand<<"\n";
  //   gaitSchedulePtr_->insertModeSequenceTemplate(gaitMapconst_.at(gaitCommand),finalTime, timeHorizon);
  // }
  // else if(leguitility[3]>=0.5 &&  gaitCommand !="stance")
  // {
  //   gaitCommand ="stance";
  //   std::cerr << "leg uitility" << leguitility[3] << "gaitCommand"<<gaitCommand<<"\n";
  //   gaitSchedulePtr_->insertModeSequenceTemplate(gaitMapconst_.at(gaitCommand),finalTime, timeHorizon);
  // }
}

}  // namespace legged_robot
}
