/**
 * @file types.cpp
 * @brief Tesseracts Collision Common Types
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_collision/core/types.h>

namespace tesseract_collision
{
void ContactResult::clear()
{
  distance = std::numeric_limits<double>::max();
  nearest_points[0].setZero();
  nearest_points[1].setZero();
  nearest_points_local[0].setZero();
  nearest_points_local[1].setZero();
  transform[0] = Eigen::Isometry3d::Identity();
  transform[1] = Eigen::Isometry3d::Identity();
  link_names[0] = "";
  link_names[1] = "";
  shape_id[0] = -1;
  shape_id[1] = -1;
  subshape_id[0] = -1;
  subshape_id[1] = -1;
  type_id[0] = 0;
  type_id[1] = 0;
  normal.setZero();
  cc_time[0] = -1;
  cc_time[1] = -1;
  cc_type[0] = ContinuousCollisionType::CCType_None;
  cc_type[1] = ContinuousCollisionType::CCType_None;
  cc_transform[0] = Eigen::Isometry3d::Identity();
  cc_transform[1] = Eigen::Isometry3d::Identity();
  single_contact_point = false;
}

ContactRequest::ContactRequest(ContactTestType type) : type(type) {}

std::size_t flattenMoveResults(ContactResultMap&& m, ContactResultVector& v)
{
  v.clear();
  v.reserve(m.size());
  for (const auto& mv : m)
    std::move(mv.second.begin(), mv.second.end(), std::back_inserter(v));

  return v.size();
}

std::size_t flattenCopyResults(const ContactResultMap& m, ContactResultVector& v)
{
  v.clear();
  v.reserve(m.size());
  for (const auto& mv : m)
    std::copy(mv.second.begin(), mv.second.end(), std::back_inserter(v));

  return v.size();
}

std::size_t flattenResults(ContactResultMap&& m, ContactResultVector& v) { return flattenMoveResults(std::move(m), v); }

ContactTestData::ContactTestData(const std::vector<std::string>& active,
                                 CollisionMarginData collision_margin_data,
                                 IsContactAllowedFn fn,
                                 ContactRequest req,
                                 ContactResultMap& res)
  : active(&active)
  , collision_margin_data(std::move(collision_margin_data))
  , fn(std::move(fn))
  , req(std::move(req))
  , res(&res)
{
}

ContactManagerConfig::ContactManagerConfig(double default_margin)
  : margin_data_override_type(CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN), margin_data(default_margin)
{
}

CollisionCheckConfig::CollisionCheckConfig(double default_margin,
                                           ContactRequest request,
                                           CollisionEvaluatorType type,
                                           double longest_valid_segment_length)
  : contact_manager_config(default_margin)
  , contact_request(std::move(request))
  , type(type)
  , longest_valid_segment_length(longest_valid_segment_length)
{
}

ContactTrajectorySubstepResults::ContactTrajectorySubstepResults(const int& substep_number,
                                                                 Eigen::VectorXd start_state,
                                                                 Eigen::VectorXd end_state)
  : substep(substep_number), state0(std::move(start_state)), state1(std::move(end_state))
{
}

ContactTrajectorySubstepResults::ContactTrajectorySubstepResults(const int& substep_number, Eigen::VectorXd state)
  : substep(substep_number), state0(std::move(state)), state1(std::move(state))
{
}

int ContactTrajectorySubstepResults::numContacts() const { return static_cast<int>(contacts.size()); }

tesseract_collision::ContactResultVector ContactTrajectorySubstepResults::worstCollision() const
{
  tesseract_collision::ContactResultVector worst_collision;
  double worst_distance = std::numeric_limits<double>::max();
  for (const auto& collision : contacts)
  {
    
    if (collision.second.front().distance < worst_distance)
    {
      worst_distance = collision.second.front().distance;
      worst_collision = collision.second;
    }
  }
  return worst_collision;
}

ContactTrajectoryStepResults::ContactTrajectoryStepResults(const int& step_number,
                                                           Eigen::VectorXd start_state,
                                                           Eigen::VectorXd end_state,
                                                           const int& num_substeps)
  : step(step_number), state0(std::move(start_state)), state1(std::move(end_state)), total_substeps(num_substeps)
{
  substeps.resize(static_cast<std::size_t>(num_substeps));
}

ContactTrajectoryStepResults::ContactTrajectoryStepResults(const int& step_number, const Eigen::VectorXd& state)
  : step(step_number), state0(state), state1(state), total_substeps(1)
{
  substeps.resize(static_cast<std::size_t>(1));
}

int ContactTrajectoryStepResults::numSubsteps() const { return static_cast<int>(substeps.size()); }

int ContactTrajectoryStepResults::numContacts() const
{
  int num_contacts = 0;
  for (const auto& substep : substeps)
    num_contacts += substep.numContacts();
  return num_contacts;
}

ContactTrajectorySubstepResults ContactTrajectoryStepResults::worstSubstep() const
{
  ContactTrajectorySubstepResults worst_substep;
  double worst_distance = std::numeric_limits<double>::max();
  for (const auto& substep : substeps)
  {
    tesseract_collision::ContactResultVector substep_worst_collision = substep.worstCollision();
    if (substep_worst_collision.front().distance < worst_distance)
    {
      worst_distance = substep_worst_collision.front().distance;
      worst_substep = substep;
    }
  }
  return worst_substep;
}

tesseract_collision::ContactResultVector ContactTrajectoryStepResults::worstCollision() const
{
  tesseract_collision::ContactResultVector worst_collision = worstSubstep().worstCollision();
  return worst_collision;
}

ContactTrajectorySubstepResults ContactTrajectoryStepResults::mostCollisionsSubstep() const
{
  int most_contacts = 0;
  ContactTrajectorySubstepResults most_collisions_substep;
  for (const auto& substep : substeps)
  {
    if (substep.numContacts() > most_contacts)
    {
      most_contacts = substep.numContacts();
      most_collisions_substep = substep;
    }
  }
  return most_collisions_substep;
}

ContactTrajectoryResults::ContactTrajectoryResults(std::vector<std::string> j_names, const int& num_steps)
  : joint_names(std::move(j_names)), total_steps(num_steps)
{
  steps.resize(static_cast<std::size_t>(num_steps));
}

int ContactTrajectoryResults::numSteps() const { return static_cast<int>(steps.size()); }

int ContactTrajectoryResults::numContacts() const
{
  int num_contacts = 0;
  for (const auto& step : steps)
    num_contacts += step.numContacts();
  return num_contacts;
}

ContactTrajectoryStepResults ContactTrajectoryResults::worstStep() const
{
  ContactTrajectoryStepResults worst_step;
  double worst_distance = std::numeric_limits<double>::max();
  for (const auto& step : steps)
  {
    tesseract_collision::ContactResultVector step_worst_collision = step.worstCollision();
    if (step_worst_collision.front().distance < worst_distance)
    {
      worst_distance = step_worst_collision.front().distance;
      worst_step = step;
    }
  }
  return worst_step;
}

tesseract_collision::ContactResultVector ContactTrajectoryResults::worstCollision() const
{
  tesseract_collision::ContactResultVector worst_collision = worstStep().worstCollision();
  return worst_collision;
}

ContactTrajectoryStepResults ContactTrajectoryResults::mostCollisionsStep() const
{
  int most_contacts = 0;
  ContactTrajectoryStepResults most_collisions_step;
  for (const auto& step : steps)
  {
    if (step.numContacts() > most_contacts)
    {
      most_contacts = step.numContacts();
      most_collisions_step = step;
    }
  }
  return most_collisions_step;
}
std::stringstream ContactTrajectoryResults::trajectoryCollisionResultsTable() const
{
  // Possible multiple contacts for every substep
  // For every contact need to display contact distance, link1, link2
  // For every substep with a contact need to display substep #/total, all contacts
  // For every step need to display joint names, state0, state1, all substeps
  // No seperation between collision lines
  // Dashed  (---) line seperating substeps
  // Star (***) line seperating steps
  std::stringstream ss;

  if (numContacts() == 0)
  {
    ss << "No contacts detected" << std::endl;
    return ss;
  }

  int step_details_width = 0;
  int substep_details_width = 0;

  // First need to determine the width of every column, should be a space on either side of each
  // Step is displayed as (step)/(total number of steps), example: 2/23
  std::string step_title = "STEP";
  int longest_steps_width = 2 + static_cast<int>(step_title.size());
  int number_steps_digits = static_cast<int>(std::log10(steps.back().step)) + 1;
  // *2 for either side, plus 1 for '/', plus 2 for spaces
  int width_steps_display = number_steps_digits * 2 + 3;
  if (width_steps_display > longest_steps_width)
    longest_steps_width = width_steps_display;

  step_details_width += longest_steps_width;

  // Joint Names can vary widely
  std::string joint_name_title = "JOINT NAMES";
  int longest_joint_name_width = static_cast<int>(joint_name_title.size()) + 2;
  for (const auto& name : joint_names)
  {
    if (static_cast<int>(name.size()) + 2 > longest_joint_name_width)
      longest_joint_name_width = static_cast<int>(name.size()) + 2;
  }

  step_details_width += longest_joint_name_width;

  // State0 and State1 we will truncate all values to be to 4 decimals of precision,
  // important to add 1 to the length of negative values to account for the sign
  std::string state0_title = "STATE0";
  std::string state1_title = "STATE1";
  int longest_state0_width = 9;  // Default negative sign, number, decimal point, four places, plus space either side
  int longest_state1_width = 9;
  for (const auto& step : steps)
  {
    for (int i = 0; i < static_cast<int>(step.state0.size()); i++)
    {
      double state0_value = step.state0(i);
      if (state0_value < 0)
      {
        state0_value *= -1;
      }
      int state0_number_digits_left_decimal = static_cast<int>(std::log10(state0_value)) + 1;
      if (state0_number_digits_left_decimal + 7 > longest_state0_width)
        longest_state0_width =
            state0_number_digits_left_decimal + 7;  // + 4 after decimal + 2 for spaces either side + 1 for decimal

      double state1_value = step.state1(i);
      if (state1_value < 0)
      {
        state1_value *= -1;
      }
      int state1_number_digits_left_decimal = static_cast<int>(std::log10(state1_value)) + 1;
      if (state1_number_digits_left_decimal + 7 > longest_state1_width)
        longest_state1_width =
            state1_number_digits_left_decimal + 7;  // + 4 after decimal + 2 for spaces either side + 1 for decimal
    }
  }

  step_details_width += longest_state0_width;
  step_details_width += longest_state1_width;

  // Substep will almost certainly be the width of substep, but still check
  std::string substep_title = "SUBSTEP";
  int longest_substep_width = 2 + static_cast<int>(substep_title.size());
  for (const auto& step : steps)
  {
    // Check to make sure there are value, could be empty if checking for first collision
    if (step.numSubsteps() == 0)
      continue;

    // Substep is displayed as (substep)/(total number of substeps), example: 5/7
    // so length will be 2*(max substep width) + 1
    int number_digits = static_cast<int>(std::log10(step.substeps.size())) + 1;
    int width = 2 * number_digits + 3;
    if (width > longest_substep_width)
      longest_substep_width = width;
  }

  substep_details_width += longest_substep_width;

  // Link1 and Link2 will each be the width of the widest link name in that calumn
  std::string link1_title = "LINK1";
  std::string link2_title = "LINK2";
  int longest_link1_width = static_cast<int>(link1_title.size()) + 2;
  int longest_link2_width = static_cast<int>(link2_title.size()) + 2;
  for (const auto& step : steps)
  {
    for (const auto& substep : step.substeps)
    {
      for (const auto& collision : substep.contacts)
      {
        std::string link1_name = collision.second.front().link_names[0];
        if (static_cast<int>(link1_name.size()) + 2 > longest_link1_width)
          longest_link1_width = static_cast<int>(link1_name.size()) + 2;

        std::string link2_name = collision.second.front().link_names[1];
        if (static_cast<int>(link2_name.size()) + 2 > longest_link2_width)
          longest_link2_width = static_cast<int>(link2_name.size()) + 2;
      }
    }
  }

  substep_details_width += longest_link1_width;
  substep_details_width += longest_link2_width;

  // Distance will also be truncated at 4 decimal points of precision, shouldn't need more
  // than 0.1 mm of precision
  // Assumming "DISTANCE" is the widest text, also doesn't matter because this is the last column
  std::string distance_title = "DISTANCE";
  int longest_distance_width = static_cast<int>(distance_title.size()) + 2;

  substep_details_width += longest_distance_width;

  // Construct strings for displaying info on a new state and new substate
  std::string new_step_string(static_cast<std::size_t>(step_details_width), '*');
  new_step_string += "|";
  new_step_string += std::string(static_cast<std::size_t>(substep_details_width), '*');
  std::string new_substep_string(static_cast<std::size_t>(substep_details_width), '-');

  // Start making the table
  // Start on new line to avoid offset by anythnig on previous line
  ss << std::endl;
  // Make the header
  ss << std::setw(longest_steps_width) << step_title << std::setw(longest_joint_name_width) << joint_name_title
     << std::setw(longest_state0_width) << state0_title << std::setw(longest_state1_width) << state1_title << "|"
     << std::setw(longest_substep_width) << substep_title << std::setw(longest_link1_width) << link1_title
     << std::setw(longest_link2_width) << link2_title << std::setw(longest_distance_width) << distance_title
     << std::endl;

  ss << new_step_string << std::endl;

  for (const auto& step : steps)
  {
    // Check if there are contacts in this step
    if (step.numContacts() == 0)
      continue;

    // Create string for stating the step number, repeated on every line of this step. example: 2/23
    std::string step_number_string = std::to_string(step.step) + "/" + std::to_string(total_steps);
    int line_number = 0;
    for (const auto& substep : step.substeps)
    {
      // Check if there are contacts in this substep
      if (substep.numContacts() == 0)
        continue;

      // Create string for stating the substep number, repeated on every line of this substep
      std::string substep_string = std::to_string(substep.substep) + "/" + std::to_string(step.total_substeps);

      // Iterate over every collision in this substep
      for (const auto& collision : substep.contacts)
      {
        // Write the current substep string
        ss << std::setw(longest_steps_width) << step_number_string;

        // Check if we still need to be adding to the joint state information
        if (line_number < static_cast<int>(joint_names.size()))
        {
          ss << std::setprecision(4) << std::fixed;
          ss << std::setw(longest_joint_name_width) << joint_names[static_cast<std::size_t>(line_number)];
          ss << std::setw(longest_state0_width) << step.state0(line_number);
          ss << std::setw(longest_state1_width) << step.state1(line_number);
        }
        else
        {
          // Add blank spaces once done writing joint states
          ss << std::setw(longest_joint_name_width) << " " << std::setw(longest_state0_width) << " "
             << std::setw(longest_state1_width) << " ";
        }
        // Add vertical bar
        ss << "|";

        // Add specific contact information
        ss << std::setw(longest_substep_width) << substep_string;
        ss << std::setw(longest_link1_width) << collision.second.front().link_names[0];
        ss << std::setw(longest_link2_width) << collision.second.front().link_names[1];
        ss << std::setprecision(10) << std::setw(16)  <<  collision.second.front().distance << std::setprecision(4);
        ss << std::endl;
        line_number++;
      }

      // Make new line for seperator between substates
      ss << std::setw(longest_steps_width) << step_number_string;
      if (line_number < static_cast<int>(joint_names.size()))
      {
        ss << std::setw(longest_joint_name_width) << joint_names[static_cast<std::size_t>(line_number)];
        ss << std::setw(longest_state0_width) << step.state0(line_number);
        ss << std::setw(longest_state1_width) << step.state1(line_number);
      }
      else
      {
        ss << std::setw(longest_joint_name_width) << " " << std::setw(longest_state0_width) << " "
           << std::setw(longest_state1_width) << " ";
      }
      ss << "|";
      ss << new_substep_string;
      ss << std::endl;
      line_number++;
    }

    // Finish writing joint state if necessary
    while (line_number < static_cast<int>(joint_names.size()))
    {
      ss << std::setw(longest_steps_width) << step_number_string;
      ss << std::setw(longest_joint_name_width) << joint_names[static_cast<std::size_t>(line_number)];
      ss << std::setw(longest_state0_width) << step.state0(line_number);
      ss << std::setw(longest_state1_width) << step.state1(line_number);
      ss << "|" << std::endl;
      line_number++;
    }
    ss << new_step_string << std::endl;
  }
  return ss;
}

}  // namespace tesseract_collision
