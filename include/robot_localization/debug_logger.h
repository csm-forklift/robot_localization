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

#ifndef ROBOT_LOCALIZATION_DEBUG_LOGGER_H
#define ROBOT_LOCALIZATION_DEBUG_LOGGER_H

#include <fstream>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>

#define RL_DEBUG(msg)\
{\
  boost::mutex::scoped_lock lock(DebugLogger::getInstance().debugStreamMutex_);\
  if (DebugLogger::getInstance().initialized_)\
{\
  DebugLogger::getInstance().debugStream_<<msg;\
}\
}

namespace RobotLocalization
{
  //! @brief Thead safe Singleton class to log debug data to file
  class DebugLogger
  {
  private:
    DebugLogger()
      : initialized_(false)
    {
    }

    DebugLogger(DebugLogger const&);    // Not implemented
    void operator=(DebugLogger const&); // Not implemented

  public:
    static DebugLogger& getInstance()
    {
      static DebugLogger    instance;
      return instance;
    }

    void setDebugFile(const std::string& debugOutFile)
    {
      boost::mutex::scoped_lock lock(debugStreamMutex_);
      if (!initialized_)
      {
        try
        {
          debugStream_.open(debugOutFile.c_str());
          // Make sure we succeeded
          if (debugStream_.is_open())
          {
            initialized_ = true;
          }
          else
          {
            ROS_WARN_STREAM("DebugLogger - unable to create debug output file " << debugOutFile);
          }
        }
        catch(const std::exception &e)
        {
          ROS_WARN_STREAM("DebugLogger - unable to create debug output file" << debugOutFile
                          << ". Error was " << e.what() << "\n");
        }
      }
    }

    ~DebugLogger()
    {
      boost::mutex::scoped_lock lock(debugStreamMutex_);
      debugStream_.close();
    }

    std::ofstream debugStream_;
    boost::mutex debugStreamMutex_;
    bool initialized_;
  };
}
#endif // ROBOT_LOCALIZATION_DEBUG_LOGGER_H
