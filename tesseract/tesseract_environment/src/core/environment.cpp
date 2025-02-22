/**
 * @file environment.cpp
 * @brief Tesseract environment interface implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_collision/core/common.h>

namespace tesseract_environment
{
void Environment::setState(const std::unordered_map<std::string, double>& joints)
{
  mutex_.lock();
  state_solver_->setState(joints);
  currentStateChanged();
  mutex_.unlock();
}

void Environment::setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values)
{
  mutex_.lock();
  state_solver_->setState(joint_names, joint_values);
  currentStateChanged();
  mutex_.unlock();
}

void Environment::setState(const std::vector<std::string>& joint_names,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  mutex_.lock();
  state_solver_->setState(joint_names, joint_values);
  currentStateChanged();
  mutex_.unlock();
}

EnvState::Ptr Environment::getState(const std::unordered_map<std::string, double>& joints) const
{
  mutex_.lock();
  EnvState::Ptr state = state_solver_->getState(joints);
  mutex_.unlock();
  return state;
}

EnvState::Ptr Environment::getState(const std::vector<std::string>& joint_names,
                                    const std::vector<double>& joint_values) const
{
  mutex_.lock();
  EnvState::Ptr state = state_solver_->getState(joint_names, joint_values);
  mutex_.unlock();
  return state;
}

EnvState::Ptr Environment::getState(const std::vector<std::string>& joint_names,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  mutex_.lock();
  EnvState::Ptr state = state_solver_->getState(joint_names, joint_values);
  mutex_.unlock();

  return state;
}

bool Environment::addLink(tesseract_scene_graph::Link link)
{
  std::string joint_name = "joint_" + link.getName();
  tesseract_scene_graph::Joint joint(joint_name);
  joint.type = tesseract_scene_graph::JointType::FIXED;
  joint.child_link_name = link.getName();
  joint.parent_link_name = getRootLinkName();

  return addLink(link, joint);
}

bool Environment::addLink(tesseract_scene_graph::Link link, tesseract_scene_graph::Joint joint)
{
  mutex_.lock();
  if (scene_graph_->getLink(link.getName()) != nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) with same name as an existing link.", link.getName().c_str());
    return false;
  }

  if (scene_graph_->getJoint(joint.getName()) != nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add joint (%s) with same name as an existing joint.", joint.getName().c_str());
    return false;
  }

  if (!scene_graph_->addLink(link))
    return false;

  if (!scene_graph_->addJoint(joint))
    return false;

  if (link.collision.size() > 0)
  {
    tesseract_collision::CollisionShapesConst shapes;
    tesseract_common::VectorIsometry3d shape_poses;
    getCollisionObject(shapes, shape_poses, link);

    if (discrete_manager_ != nullptr)
      discrete_manager_->addCollisionObject(link.getName(), 0, shapes, shape_poses, true);
    if (continuous_manager_ != nullptr)
      continuous_manager_->addCollisionObject(link.getName(), 0, shapes, shape_poses, true);
  }

  ++revision_;
  commands_.push_back(
      std::make_shared<AddCommand>(scene_graph_->getLink(link.getName()), scene_graph_->getJoint(joint.getName())));

  environmentChanged();
  mutex_.unlock();

  return true;
}

bool Environment::removeLink(const std::string& name)
{
  mutex_.lock();
  if (!removeLinkHelper(name))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<RemoveLinkCommand>(name));

  environmentChanged();
  mutex_.unlock();

  return true;
}

bool Environment::moveLink(tesseract_scene_graph::Joint joint)
{
  mutex_.lock();
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getInboundJoints(joint.child_link_name);
  assert(joints.size() == 1);
  if (!scene_graph_->removeJoint(joints[0]->getName()))
    return false;

  if (!scene_graph_->addJoint(joint))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<MoveLinkCommand>(scene_graph_->getJoint(joint.getName())));

  environmentChanged();
  mutex_.unlock();

  return true;
}

tesseract_scene_graph::Link::ConstPtr Environment::getLink(const std::string& name) const
{
  mutex_.lock();
  tesseract_scene_graph::Link::ConstPtr link = scene_graph_->getLink(name);
  mutex_.unlock();

  return link;
}

bool Environment::removeJoint(const std::string& name)
{
  mutex_.lock();
  if (scene_graph_->getJoint(name) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove Joint (%s) that does not exist", name.c_str());
    return false;
  }

  std::string target_link_name = scene_graph_->getTargetLink(name)->getName();

  if (!removeLinkHelper(target_link_name))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<RemoveJointCommand>(name));

  environmentChanged();
  mutex_.unlock();

  return true;
}

bool Environment::moveJoint(const std::string& joint_name, const std::string& parent_link)
{
  mutex_.lock();
  if (!scene_graph_->moveJoint(joint_name, parent_link))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<MoveJointCommand>(joint_name, parent_link));

  environmentChanged();
  mutex_.unlock();

  return true;
}

bool Environment::changeJointOrigin(const std::string& joint_name, const Eigen::Isometry3d& new_origin)
{
  mutex_.lock();
  if (!scene_graph_->changeJointOrigin(joint_name, new_origin))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<ChangeJointOriginCommand>(joint_name, new_origin));

  environmentChanged();
  mutex_.unlock();

  return true;
}

void Environment::setLinkCollisionEnabled(const std::string& name, bool enabled)
{
  mutex_.lock();
  if (discrete_manager_ != nullptr)
  {
    if (enabled)
      discrete_manager_->enableCollisionObject(name);
    else
      discrete_manager_->disableCollisionObject(name);
  }

  if (continuous_manager_ != nullptr)
  {
    if (enabled)
      continuous_manager_->enableCollisionObject(name);
    else
      continuous_manager_->disableCollisionObject(name);
  }

  scene_graph_->setLinkCollisionEnabled(name, enabled);

  ++revision_;
  commands_.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(name, enabled));
  mutex_.unlock();
}

bool Environment::getLinkCollisionEnabled(const std::string& name) const
{
  mutex_.lock();
  bool enabled = scene_graph_->getLinkCollisionEnabled(name);
  mutex_.unlock();

  return enabled;
}

void Environment::setLinkVisibility(const std::string& name, bool visibility)
{
  mutex_.lock();
  scene_graph_->setLinkVisibility(name, visibility);

  ++revision_;
  commands_.push_back(std::make_shared<ChangeLinkVisibilityCommand>(name, visibility));
  mutex_.unlock();
}

bool Environment::getLinkVisibility(const std::string& name) const
{
  mutex_.lock();
  bool visible = scene_graph_->getLinkVisibility(name);
  mutex_.unlock();

  return visible;
}

void Environment::addAllowedCollision(const std::string& link_name1,
                                      const std::string& link_name2,
                                      const std::string& reason)
{
  mutex_.lock();
  scene_graph_->addAllowedCollision(link_name1, link_name2, reason);

  ++revision_;
  commands_.push_back(std::make_shared<AddAllowedCollisionCommand>(link_name1, link_name2, reason));
  mutex_.unlock();
}

void Environment::removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
{
  mutex_.lock();
  scene_graph_->removeAllowedCollision(link_name1, link_name2);

  ++revision_;
  commands_.push_back(std::make_shared<RemoveAllowedCollisionCommand>(link_name1, link_name2));
  mutex_.unlock();
}

void Environment::removeAllowedCollision(const std::string& link_name)
{
  mutex_.lock();
  scene_graph_->removeAllowedCollision(link_name);

  ++revision_;
  commands_.push_back(std::make_shared<RemoveAllowedCollisionLinkCommand>(link_name));
  mutex_.unlock();
}

tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr Environment::getAllowedCollisionMatrix() const
{
  mutex_.lock();
  tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr acm = scene_graph_->getAllowedCollisionMatrix();
  mutex_.unlock();

  return acm;
}

tesseract_scene_graph::Joint::ConstPtr Environment::getJoint(const std::string& name) const
{
  mutex_.lock();
  tesseract_scene_graph::Joint::ConstPtr joint = scene_graph_->getJoint(name);
  mutex_.unlock();

  return joint;
}

Eigen::VectorXd Environment::getCurrentJointValues() const
{
  mutex_.lock();
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(active_joint_names_.size()));
  for (auto j = 0u; j < active_joint_names_.size(); ++j)
  {
    jv(j) = current_state_->joints[active_joint_names_[j]];
  }
  mutex_.unlock();

  return jv;
}

Eigen::VectorXd Environment::getCurrentJointValues(const std::vector<std::string>& joint_names) const
{
  mutex_.lock();
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(joint_names.size()));
  for (auto j = 0u; j < joint_names.size(); ++j)
  {
    jv(j) = current_state_->joints[joint_names[j]];
  }
  mutex_.unlock();

  return jv;
}

tesseract_common::VectorIsometry3d Environment::getLinkTransforms() const
{
  mutex_.lock();
  tesseract_common::VectorIsometry3d link_tfs;
  link_tfs.resize(link_names_.size());
  for (const auto& link_name : link_names_)
  {
    link_tfs.push_back(current_state_->transforms[link_name]);
  }
  mutex_.unlock();

  return link_tfs;
}

const Eigen::Isometry3d& Environment::getLinkTransform(const std::string& link_name) const
{
  mutex_.lock();
  const Eigen::Isometry3d& tf = current_state_->transforms[link_name];
  mutex_.unlock();

  return tf;
}

StateSolver::Ptr Environment::getStateSolver() const
{
  mutex_.lock();
  StateSolver::Ptr state_solver = state_solver_->clone();
  mutex_.unlock();

  return state_solver;
}

void Environment::getCollisionObject(tesseract_collision::CollisionShapesConst& shapes,
                                     tesseract_common::VectorIsometry3d& shape_poses,
                                     const tesseract_scene_graph::Link& link) const
{
  for (const auto& c : link.collision)
  {
    // Need to force convex hull TODO: Remove after URDF dom has been updated.
    if (c->geometry->getType() == tesseract_geometry::MESH)
    {
      // This is required because convex hull cannot have multiple faces on the same plane.
      std::shared_ptr<tesseract_common::VectorVector3d> ch_verticies(new tesseract_common::VectorVector3d());
      std::shared_ptr<Eigen::VectorXi> ch_faces(new Eigen::VectorXi());
      int ch_num_faces = tesseract_collision::createConvexHull(
          *ch_verticies,
          *ch_faces,
          *(std::static_pointer_cast<const tesseract_geometry::Mesh>(c->geometry)->getVertices()));
      shapes.push_back(tesseract_geometry::ConvexMesh::Ptr(
          new tesseract_geometry::ConvexMesh(ch_verticies, ch_faces, ch_num_faces)));
    }
    else
    {
      shapes.push_back(c->geometry);
    }
    shape_poses.push_back(c->origin);
  }
}

bool Environment::setActiveDiscreteContactManager(const std::string& name)
{
  mutex_.lock();
  tesseract_collision::DiscreteContactManager::Ptr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Discrete manager with %s does not exist in factory!", name.c_str());
    return false;
  }

  discrete_manager_name_ = name;
  discrete_manager_ = std::move(manager);

  // Update the current state information since the contact manager has been created/set
  currentStateChanged();
  mutex_.unlock();

  return true;
}

tesseract_collision::DiscreteContactManager::Ptr Environment::getDiscreteContactManager(const std::string& name) const
{
  mutex_.lock();
  tesseract_collision::DiscreteContactManager::Ptr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Discrete manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }
  mutex_.unlock();

  return manager;
}

bool Environment::setActiveContinuousContactManager(const std::string& name)
{
  mutex_.lock();
  tesseract_collision::ContinuousContactManager::Ptr manager = getContinuousContactManagerHelper(name);

  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
    return false;
  }

  continuous_manager_name_ = name;
  continuous_manager_ = std::move(manager);

  // Update the current state information since the contact manager has been created/set
  currentStateChanged();
  mutex_.unlock();

  return true;
}

tesseract_collision::ContinuousContactManager::Ptr
Environment::getContinuousContactManager(const std::string& name) const
{
  mutex_.lock();
  tesseract_collision::ContinuousContactManager::Ptr manager = getContinuousContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }
  mutex_.unlock();

  return manager;
}

tesseract_collision::DiscreteContactManager::Ptr
Environment::getDiscreteContactManagerHelper(const std::string& name) const
{
  tesseract_collision::DiscreteContactManager::Ptr manager = discrete_factory_.create(name);
  if (manager == nullptr)
    return nullptr;

  manager->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (initialized_)
  {
    for (const auto& link : scene_graph_->getLinks())
    {
      if (link->collision.size() > 0)
      {
        tesseract_collision::CollisionShapesConst shapes;
        tesseract_common::VectorIsometry3d shape_poses;
        getCollisionObject(shapes, shape_poses, *link);
        manager->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      }
    }

    manager->setActiveCollisionObjects(active_link_names_);
  }

  return manager;
}

tesseract_collision::ContinuousContactManager::Ptr
Environment::getContinuousContactManagerHelper(const std::string& name) const
{
  tesseract_collision::ContinuousContactManager::Ptr manager = continuous_factory_.create(name);

  if (manager == nullptr)
    return nullptr;

  manager->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (initialized_)
  {
    for (const auto& link : scene_graph_->getLinks())
    {
      if (link->collision.size() > 0)
      {
        tesseract_collision::CollisionShapesConst shapes;
        tesseract_common::VectorIsometry3d shape_poses;
        getCollisionObject(shapes, shape_poses, *link);
        manager->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      }
    }

    manager->setActiveCollisionObjects(active_link_names_);
  }

  return manager;
}

void Environment::currentStateChanged()
{
  current_state_ = std::make_shared<EnvState>(*(state_solver_->getCurrentState()));
  if (discrete_manager_ != nullptr)
    discrete_manager_->setCollisionObjectsTransform(current_state_->transforms);
  if (continuous_manager_ != nullptr)
  {
    for (const auto& tf : current_state_->transforms)
    {
      if (std::find(active_link_names_.begin(), active_link_names_.end(), tf.first) != active_link_names_.end())
      {
        continuous_manager_->setCollisionObjectsTransform(tf.first, tf.second, tf.second);
      }
      else
      {
        continuous_manager_->setCollisionObjectsTransform(tf.first, tf.second);
      }
    }
  }
}

void Environment::environmentChanged()
{
  // Update link names
  std::vector<tesseract_scene_graph::Link::ConstPtr> links = scene_graph_->getLinks();
  link_names_.clear();
  link_names_.reserve(links.size());
  for (const auto& link : links)
    link_names_.push_back(link->getName());

  // Update joint names and active joint name
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getJoints();
  active_joint_names_.clear();
  joint_names_.clear();
  joint_names_.reserve(joints.size());
  for (const auto& joint : joints)
  {
    joint_names_.push_back(joint->getName());

    // UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    if (joint->type == tesseract_scene_graph::JointType::REVOLUTE ||
        joint->type == tesseract_scene_graph::JointType::PRISMATIC)
      active_joint_names_.push_back(joint->getName());
  }

  // Update active link names
  active_link_names_.clear();
  getActiveLinkNamesRecursive(active_link_names_, scene_graph_, scene_graph_->getRoot(), false);

  if (discrete_manager_ != nullptr)
    discrete_manager_->setActiveCollisionObjects(active_link_names_);
  if (continuous_manager_ != nullptr)
    continuous_manager_->setActiveCollisionObjects(active_link_names_);

  state_solver_->onEnvironmentChanged(commands_);

  currentStateChanged();
}

bool Environment::removeLinkHelper(const std::string& name)
{
  if (scene_graph_->getLink(name) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove link (%s) that does not exist", name.c_str());
    return false;
  }
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getInboundJoints(name);
  assert(joints.size() <= 1);

  // get child link names to remove
  std::vector<std::string> child_link_names = scene_graph_->getLinkChildrenNames(name);

  scene_graph_->removeLink(name);
  if (discrete_manager_ != nullptr)
    discrete_manager_->removeCollisionObject(name);
  if (continuous_manager_ != nullptr)
    continuous_manager_->removeCollisionObject(name);

  for (const auto& link_name : child_link_names)
  {
    scene_graph_->removeLink(link_name);

    if (discrete_manager_ != nullptr)
      discrete_manager_->removeCollisionObject(link_name);
    if (continuous_manager_ != nullptr)
      continuous_manager_->removeCollisionObject(link_name);
  }

  return true;
}

}  // namespace tesseract_environment
