/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <class_loader/class_loader.hpp>
#include <ros/console.h>

// To use struct JerkLimits 
// #include <moveit/trajectory_processing/ruckig_traj_smoothing.h>


namespace default_planner_request_adapters
{
using namespace trajectory_processing;

/** @brief This adapter uses the time-optimal trajectory generation method */
class AddRuckigTrajectorySmoothing : public planning_request_adapter::PlanningRequestAdapter
{
public:
  AddRuckigTrajectorySmoothing() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const ros::NodeHandle& /*nh*/) override
  {
    // Correct way should be to grab jerk limits from the robot_model/joint_model
    // however as limits are passed using a message this needs to 
    // update moveit_msgs package to include jerk limits (done in ros2)
    // so here we grab the limits manually and pass them to ruckig through
    // the RuckigSmoothing object
    ros::NodeHandle n("");
    XmlRpc::XmlRpcValue joint_limits;
    if (n.getParam("robot_description_planning/joint_limits", joint_limits))
    {
      int i = 0;
      jerk_limits.resize(joint_limits.size());
      // iterate over each joint to check if jerk limit exists and get its values
      for (auto& jl: joint_limits){

        // The jl is a pair type, the jl.first and jl.second can be used
        XmlRpc::XmlRpcValue ja_limits = jl.second;
        if (ja_limits.hasMember("max_jerk")) {
          double value;
          if (ja_limits["max_jerk"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            value = ja_limits["max_jerk"];
          if (ja_limits["max_jerk"].getType() == XmlRpc::XmlRpcValue::TypeInt)
          {
            value = (int) ja_limits["max_jerk"];
          }
          jerk_limits[i].max_jerk = value;
          jerk_limits[i].has_jerk_limits = true;
        }

        if (ja_limits.hasMember("has_jerk_limits")){
          jerk_limits[i].has_jerk_limits = ja_limits["has_jerk_limits"];
        }

        // ROS_INFO_STREAM("In " << jl.first + " the has_jerk_limits is: " << jerk_limits[i].has_jerk_limits);
        // ROS_INFO_STREAM("In " << jl.first + " the max_jerk is: " << jerk_limits[i].max_jerk);
        i=i+1;
      }
      // show info about found joint limits
      ROS_INFO_STREAM("Jerk joint limits defined. Read from joint_limits.yaml!");
    }
    else{
      // show warning about joint limits not found
      ROS_WARN("No joint limits defined. Fill and load joint_limits.yaml!");
    }
  }

  std::string getDescription() const override
  {
    return "Add Ruckig trajectory smoothing.";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {
      if (!smoother_.applySmoothing(*res.trajectory_, jerk_limits, req.max_velocity_scaling_factor,
                                    req.max_acceleration_scaling_factor))
      {
        result = false;
      }
    }

    return result;
  }

private:
  std::vector<JerkLimits> jerk_limits;
  RuckigSmoothing smoother_;
};

}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AddRuckigTrajectorySmoothing,
                            planning_request_adapter::PlanningRequestAdapter);
