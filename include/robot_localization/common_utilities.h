#ifndef ROBOT_LOCALIZATION_COMMON_UTILITIES_H
#define ROBOT_LOCALIZATION_COMMON_UTILITIES_H

#include<fstream>
#include<boost/thread/mutex.hpp>
#include<ros/ros.h>

#define RL_DEBUG(msg)\
{\
  boost::mutex::scoped_lock lock(CommonUtilities::DebugLogger::getInstance().debugStreamMutex_);\
  if (CommonUtilities::DebugLogger::getInstance().initialized_)\
  {\
    CommonUtilities::DebugLogger::getInstance().debugStream_<<msg;\
  }\
}

namespace RobotLocalization
{
  namespace CommonUtilities
  {
    //! @brief Thead safe Singleton class to log debug data to file
    class DebugLogger
    {
    private:
      DebugLogger(): initialized_(false)
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
}
#endif // ROBOT_LOCALIZATION_COMMON_UTILITIES_H
