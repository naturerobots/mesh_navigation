/*
 *  Copyright 2019, Sebastian Pütz
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
 *  mesh_recovery_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_MESH_NAV__COSTMAP_RECOVERY_EXECUTION_H
#define MBF_MESH_NAV__COSTMAP_RECOVERY_EXECUTION_H

#include <mbf_abstract_nav/abstract_recovery_execution.h>
#include <mbf_mesh_nav/MoveBaseFlexConfig.h>
#include <mbf_mesh_core/mesh_recovery.h>
#include <mesh_2d/mesh_2d_ros.h>


namespace mbf_mesh_nav
{
/**
 * @brief The MeshRecoveryExecution binds a local and a global mesh to the AbstractRecoveryExecution and uses the
 *        nav_core/MeshRecovery class as base plugin interface. This class makes move_base_flex compatible to the old move_base.
 *
 * @ingroup recovery_execution move_base_server
 */
class MeshRecoveryExecution : public mbf_abstract_nav::AbstractRecoveryExecution
{

public:
  typedef boost::shared_ptr<mesh_2d::Mesh2DROS> MeshPtr;
  typedef boost::shared_ptr<MeshRecoveryExecution> Ptr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common tf listener
   * @param global_mesh Shared pointer to the global mesh.
   * @param local_mesh Shared pointer to the local mesh.
   */
  MeshRecoveryExecution(
      const std::string name,
      const mbf_mesh_core::MeshRecovery::Ptr &recovery_ptr,
      const TFPtr &tf_listener_ptr,
      MeshPtr &global_mesh,
      MeshPtr &local_mesh,
      const MoveBaseFlexConfig &config,
      boost::function<void()> setup_fn,
      boost::function<void()> cleanup_fn);
  /**
   * Destructor
   */
  virtual ~MeshRecoveryExecution();

protected:

  //! Shared pointer to the global mesh
  MeshPtr &global_mesh_;

  //! Shared pointer to thr local mesh
  MeshPtr &local_mesh_;

private:

  mbf_abstract_nav::MoveBaseFlexConfig toAbstract(const MoveBaseFlexConfig &config);
};

} /* namespace mbf_mesh_nav */

#endif /* MBF_MESH_NAV__COSTMAP_RECOVERY_EXECUTION_H */
