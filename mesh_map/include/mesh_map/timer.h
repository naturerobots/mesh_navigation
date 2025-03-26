/*
 *  Copyright 2025, Justus Braun
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
 *    Justus Braun <jubraun@uni-osnabrueck.de>
 *
 */

#ifndef MESH_MAP__TIMER_H
#define MESH_MAP__TIMER_H

#include <chrono>
#include <string>
#include <fstream>
#include <mutex>

namespace mesh_map
{

/**
* @brief Provides a facility to log layer update timings to a file.
*/
class LayerTimer
{
public:
  
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Duration = std::chrono::nanoseconds;

  /**
  * @brief Enable logging globally
  */
  static void enable();

  /**
  * @brief Disable logging globally
  */
  static void disable();
  
  /**
  * @brief Record update duration.
  *
  * @param layer The layer name
  * @param timestamp The timestamp to associate with this record
  * @param locking Time spend waiting for locks
  * @param update Time spend updating the layer
  * @param notify Time spend calling AbstractLayer::notifyChange
  *
  */
  static void recordUpdateDuration(
    const std::string& layer,
    const TimePoint& timestamp,
    const Duration& locking,
    const Duration& update,
    const Duration& notify
  );

private:

  volatile bool enabled_ = false;
  std::ofstream file_;
  std::mutex mutex_;
  
  // C++11 guarantees that static instances in this form are initialized in
  // a thread safe manner.
  // https://en.cppreference.com/w/cpp/language/storage_duration#Static_block_variables
  static LayerTimer& instance()
  {
    static LayerTimer instance;
    return instance;
  }
};

} // namespace mesh_map

#endif 
