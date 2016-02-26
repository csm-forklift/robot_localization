/*
 * Copyright (c) 2015, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOT_LOCALIZATION_TIME_TO_PUBLISH_H
#define ROBOT_LOCALIZATION_TIME_TO_PUBLISH_H

#include <ros/ros.h>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread.hpp>

namespace RobotLocalization
{

  //! @brief Class to determine when to publish in order to
  //! maintain a constant publishing rate
  class TimeToPublish
  {
  public:
    //! @brief Constructor
    //!
    TimeToPublish(const double frequency_ = 50.0);

    //! @brief Set the publishing period
    //!
    void setFrequency(const double frequency_);

    //! @brief True if it's time to publish
    //!
    bool operator()();

    //! @brief Set the condition variable to be notified when it is time to publish
    //!
    void setConditionVariable(boost::condition_variable& condition);

  private:
    //! @brief frequency to maintain
    //!
    double frequency_;

    //! @brief True if it's time to publish
    //!
    bool isTime_;

    //! @brief Condition variable to be notified on time to publish
    //!
    boost::shared_ptr<boost::condition_variable> condition_;

    //! @brief Mutex to protect access to isTime_
    //!
    boost::mutex mutex_;

    //! @brief Thread that runs the timing loop
    //!
    boost::shared_ptr<boost::thread> thread_;

    //! @brief Function that runs the timing loop
    //!
    void run();
  };

  TimeToPublish::TimeToPublish(const double frequency)
    : frequency_(frequency)
    , isTime_(false)
  {
  }

  void TimeToPublish::setFrequency(const double frequency)
  {
    frequency_ = frequency;

    // Stop timing thread if running
    if (thread_)
    {
      thread_->interrupt();
      thread_->join();
    }

    thread_.reset(new boost::thread(&TimeToPublish::run, this));
  }

  void TimeToPublish::setConditionVariable(boost::condition_variable& condition)
  {
    boost::mutex::scoped_lock lock(mutex_);
    condition_.reset(&condition);
  }

  bool TimeToPublish::operator()()
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (isTime_)
    {
      isTime_ = false;
      return true;
    }

    return false;
  }

  void TimeToPublish::run()
  {
    ros::Rate rate(frequency_);

    try
    {
      while (ros::ok())
      {
        {
          boost::mutex::scoped_lock lock(mutex_);

          isTime_ = true;

          if (condition_)
          {
            condition_->notify_one();
          }
        }
        rate.sleep();
        boost::this_thread::interruption_point();
      }
    }
    catch (boost::thread_interrupted&)
    {
      ROS_DEBUG("TimeToPublish: Timing thread interrupted");
    }
  }

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_TIME_TO_PUBLISH_H
