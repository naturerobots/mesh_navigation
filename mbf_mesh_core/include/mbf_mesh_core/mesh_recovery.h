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
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_MESH_CORE__ABSTRACT_RECOVERY_H
#define MBF_MESH_CORE__ABSTRACT_RECOVERY_H

#include <string>
#include <memory>

#include <mbf_abstract_core/abstract_recovery.h>
#include <mesh_map/mesh_map.h>

namespace mbf_mesh_core
{
/**
 * @class MeshRecovery
 * @brief Provides an interface for recovery behaviors used in navigation.
 * All recovery behaviors written as plugins for the navigation stack must
 * adhere to this interface.
 */
class MeshRecovery : public mbf_abstract_core::AbstractRecovery
{
public:
  typedef std::shared_ptr<::mbf_mesh_core::MeshRecovery> Ptr;

  /**
   * @brief Runs the AbstractRecovery
   * @param message The recovery behavior could set, the message should
   * correspond to the return value
   * @return An outcome which will be hand over to the action result.
   */
  virtual uint32_t runBehavior(std::string& message) = 0;

  //! MeshRecovery cannot be constructed
  MeshRecovery() = delete;

  /**
   * @brief Virtual destructor for the interface
   */
  virtual ~MeshRecovery()
  {
  }

  /**
   * @brief Requests the recovery behavior to cancel, e.g. if it takes too much
   * time.
   * @return True if a cancel has been successfully requested, false if not
   * implemented.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Initializes the recovery plugin with a name, a tf pointer and a mesh map pointer
   * @param name The recovery behavior plugin name, defined by the user. It defines the plugins namespace
   * @param tf_ptr A shared pointer to a transformation buffer
   * @param mesh_map_ptr A shared pointer to the mesh map
   * @return true if the plugin has been initialized successfully
   */
  virtual bool initialize(const std::string& name, const boost::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                          const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr) = 0;

};
}; /* namespace mbf_mesh_core */

#endif /* MBF_MESH_CORE__ABSTRACT_RECOVERY_H */
