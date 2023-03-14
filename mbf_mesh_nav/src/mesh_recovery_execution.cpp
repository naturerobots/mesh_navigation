/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */
#include "mbf_mesh_nav/mesh_recovery_execution.h"

namespace mbf_mesh_nav
{
MeshRecoveryExecution::MeshRecoveryExecution(const std::string name, const mbf_mesh_core::MeshRecovery::Ptr& recovery_ptr,
                                             const mbf_utility::RobotInformation &robot_info, const MeshPtr& mesh_ptr,
                                             const MoveBaseFlexConfig& config)
  : AbstractRecoveryExecution(name, recovery_ptr, robot_info, toAbstract(config)), mesh_ptr_(mesh_ptr)
{
}

MeshRecoveryExecution::~MeshRecoveryExecution()
{
}

mbf_abstract_nav::MoveBaseFlexConfig MeshRecoveryExecution::toAbstract(const MoveBaseFlexConfig& config)
{
  // copy the recovery-related abstract configuration common to all MBF-based
  // navigation
  mbf_abstract_nav::MoveBaseFlexConfig abstract_config;
  abstract_config.recovery_enabled = config.recovery_enabled;
  abstract_config.recovery_patience = config.recovery_patience;
  return abstract_config;
}

} /* namespace mbf_mesh_nav */
