/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
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

#ifndef ROBOT_LOCALIZATION_MH_EKF_H
#define ROBOT_LOCALIZATION_MH_EKF_H

#include "robot_localization/filter_base.h"
#include "robot_localization/ekf.h"

#include <boost/range/algorithm/min_element.hpp>
#include <stdexcept>

#include <fstream>
#include <vector>
#include <set>
#include <queue>
#include <ros/ros.h>

namespace RobotLocalization
{

  //! @brief A Hypothesis
  //!
  //! Implementation of an extended Kalman filter (EKF). This
  //! class derives from FilterBase and overrides the predict()
  //! and correct() methods in keeping with the discrete time
  //! EKF algorithm.
  //!
  class Hypothesis: public FilterBase
  {
    public:
      typedef boost::shared_ptr<Hypothesis> Ptr;
      struct PtrComp
      {
        bool operator() (const Ptr& lhs, const Ptr& rhs)
        {
          return lhs->getWeight() < rhs->getWeight();
        }
      };

      //! @brief Constructor for the Ekf class
      //!
      //! @param[in] args - Generic argument container (not used here, but
      //! needed so that the ROS filters can pass arbitrary arguments to
      //! templated filter types).
      //!
      explicit Hypothesis(std::vector<double> args = std::vector<double>());

      //! @brief Destructor for the Ekf class
      //!
      ~Hypothesis();

      //! @brief Carries out the correct step in the predict/update cycle.
      //!
      //! @param[in] measurement - The measurement to fuse with our estimate
      //!
      void correct(const Measurement &measurement);

      //! @brief Carries out the predict step in the predict/update cycle.
      //!
      //! Projects the state and error matrices forward using a model of
      //! the vehicle's motion.
      //!
      //! @param[in] referenceTime - The time at which the prediction is being made
      //! @param[in] delta - The time step over which to predict.
      //!
      void predict(const double referenceTime, const double delta);

      double& getWeight()
      {
        return weight_;
      }

      double getWeight() const
      {
        return weight_;
      }

    private:
      double weight_;
  };


//! @brief Extended Kalman filter class
//!
//! Implementation of an extended Kalman filter (EKF). This
//! class derives from FilterBase and overrides the predict()
//! and correct() methods in keeping with the discrete time
//! EKF algorithm.
//!
class MhEkf: public FilterBase
{
  public:
    //! @brief Constructor for the Ekf class
    //!
    //! @param[in] args - Generic argument container (not used here, but
    //! needed so that the ROS filters can pass arbitrary arguments to
    //! templated filter types).
    //!
    explicit MhEkf(std::vector<double> args = std::vector<double>());

    //! @brief Destructor for the MhEkf class
    //!
    ~MhEkf();

    //! @brief Updates the set of hypotheses and activates the best one
    //!
    void updateHypotheses();

    //! @brief Carries out the correct step in the predict/update cycle. This method
    //! must be implemented by subclasses.
    //!
    //! @param[in] measurement - The measurement to fuse with the state estimate
    //!
    virtual void correct(const Measurement &measurement);

    //! @brief Returns the control vector currently being used
    //!
    //! @return The control vector
    //!
    const Eigen::VectorXd& getControl();

    //! @brief Returns the time at which the control term was issued
    //!
    //! @return The time the control vector was issued
    //!
    double getControlTime();

    //! @brief Gets the value of the debug_ variable.
    //!
    //! @return True if in debug mode, false otherwise
    //!
    bool getDebug();

    //! @brief Gets the estimate error covariance
    //!
    //! @return A copy of the estimate error covariance matrix
    //!
    const Eigen::MatrixXd& getEstimateErrorCovariance();

    //! @brief Gets the filter's initialized status
    //!
    //! @return True if we've received our first measurement, false otherwise
    //!
    bool getInitializedStatus();

    //! @brief Gets the most recent measurement time
    //!
    //! @return The time at which we last received a measurement
    //!
    double getLastMeasurementTime();

    //! @brief Gets the filter's last update time
    //!
    //! @return The time at which we last updated the filter,
    //! which can occur even when we don't receive measurements
    //!
    double getLastUpdateTime();

    //! @brief Gets the filter's predicted state, i.e., the
    //! state estimate before correct() is called.
    //!
    //! @return A constant reference to the predicted state
    //!
    const Eigen::VectorXd& getPredictedState();

    //! @brief Gets the filter's process noise covariance
    //!
    //! @return A constant reference to the process noise covariance
    //!
    const Eigen::MatrixXd& getProcessNoiseCovariance();

    //! @brief Gets the sensor timeout value (in seconds)
    //!
    //! @return The sensor timeout value
    //!
    double getSensorTimeout();

    //! @brief Gets the filter state
    //!
    //! @return A constant reference to the current state
    //!
    const Eigen::VectorXd& getState();

    //! @brief Carries out the predict step in the predict/update cycle.
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion. This method must be implemented by subclasses.
    //!
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //!
    virtual void predict(const double referenceTime, const double delta);

    //! @brief Does some final preprocessing, carries out the predict/update cycle
    //!
    //! @param[in] measurement - The measurement object to fuse into the filter
    //!
    virtual void processMeasurement(const Measurement &measurement);

    //! @brief Sets the most recent control term
    //!
    //! @param[in] control - The control term to be applied
    //! @param[in] controlTime - The time at which the control in question was received
    //!
    void setControl(const Eigen::VectorXd &control, const double controlTime);

    //! @brief Sets the control update vector and acceleration limits
    //!
    //! @param[in] updateVector - The values the control term affects
    //! @param[in] controlTimeout - Timeout value, in seconds, after which a control is considered stale
    //! @param[in] accelerationLimits - The acceleration limits for the control variables
    //! @param[in] accelerationGains - Gains applied to the control term-derived acceleration
    //! @param[in] decelerationLimits - The deceleration limits for the control variables
    //! @param[in] decelerationGains - Gains applied to the control term-derived deceleration
    //!
    void setControlParams(const std::vector<int> &updateVector, const double controlTimeout,
      const std::vector<double> &accelerationLimits, const std::vector<double> &accelerationGains,
      const std::vector<double> &decelerationLimits, const std::vector<double> &decelerationGains);

    //! @brief Sets the filter into debug mode
    //!
    //! NOTE: this will generates a lot of debug output to the provided stream.
    //! The value must be a pointer to a valid ostream object.
    //!
    //! @param[in] debug - Whether or not to place the filter in debug mode
    //! @param[in] outStream - If debug is true, then this must have a valid pointer.
    //! If the pointer is invalid, the filter will not enter debug mode. If debug is
    //! false, outStream is ignored.
    //!
    void setDebug(const bool debug, std::ostream *outStream = NULL);

    //! @brief Manually sets the filter's estimate error covariance
    //!
    //! @param[in] estimateErrorCovariance - The state to set as the filter's current state
    //!
    void setEstimateErrorCovariance(const Eigen::MatrixXd &estimateErrorCovariance);

    //! @brief Sets the filter's last measurement time.
    //!
    //! @param[in] lastMeasurementTime - The last measurement time of the filter
    //!
    void setLastMeasurementTime(const double lastMeasurementTime);

    //! @brief Sets the filter's last update time.
    //!
    //! This is used mostly for initialization purposes, as the integrateMeasurements()
    //! function will update the filter's last update time as well.
    //!
    //! @param[in] lastUpdateTime - The last update time of the filter
    //!
    void setLastUpdateTime(const double lastUpdateTime);

    //! @brief Sets the process noise covariance for the filter.
    //!
    //! This enables external initialization, which is important, as this
    //! matrix can be difficult to tune for a given implementation.
    //!
    //! @param[in] processNoiseCovariance - The STATE_SIZExSTATE_SIZE process noise covariance matrix
    //! to use for the filter
    //!
    void setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance);

    //! @brief Sets the sensor timeout
    //!
    //! @param[in] sensorTimeout - The time, in seconds, for a sensor to be
    //! considered having timed out
    //!
    void setSensorTimeout(const double sensorTimeout);

    //! @brief Sets the zero velocity threshold
    //!
    //! @param[in] zervoVelocityThreshold - The velocity below which we set process noise to zero
    //! Note a negative value will disable this feature.
    //!
    void setZeroVelocityThreshold(const double zeroVelocityThreshold);

    //! @brief Manually sets the filter's state
    //!
    //! @param[in] state - The state to set as the filter's current state
    //!
    void setState(const Eigen::VectorXd &state);

    //! @brief Ensures a given time delta is valid (helps with bag file playback issues)
    //!
    //! @param[in] delta - The time delta, in seconds, to validate
    //!
    void validateDelta(double &delta);
    
private:
    Hypothesis::Ptr active_;
    std::vector<Hypothesis::Ptr> hypotheses_;
    size_t max_hyptheses_;
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_MH_EKF_H
