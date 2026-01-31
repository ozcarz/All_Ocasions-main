#ifndef PID_H
#define PID_H

#include <cmath>
#include <algorithm>

class PID {
public:
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0,
        double integralMaxAbs = 1000.0,
        double outputMin = -100.0,
        double outputMax = 100.0,
        double filterTf = 0.0,
        double antiWindupTt = 0.2)
        : kP_(kp), kI_(ki), kD_(kd),
          integralMax_(std::fabs(integralMaxAbs)),
          outputMin_(outputMin), outputMax_(outputMax),
          filterTf_(std::max(0.0, filterTf)),
          antiWindupTt_(std::max(1e-6, antiWindupTt))
    {
        if (outputMin_ > outputMax_) std::swap(outputMin_, outputMax_);
    }

    void setGains(double kp, double ki, double kd) { kP_ = kp; kI_ = ki; kD_ = kd; }
    void setDerivativeFilter(double tf) { filterTf_ = std::max(0.0, tf); }
    void setIntegralLimit(double limit) { integralMax_ = std::fabs(limit); }

    void setOutputLimits(double minVal, double maxVal) {
        outputMin_ = minVal; outputMax_ = maxVal;
        if (outputMin_ > outputMax_) std::swap(outputMin_, outputMax_);
    }

    void setAntiWindupTime(double tt) { antiWindupTt_ = std::max(1e-6, tt); }
    void setFeedForward(double ff) { feedForward_ = ff; }
    void setSetpoint(double sp) { setpoint_ = sp; }

    double update(double measurement, double dt) {
        dt = std::max(dt, 1e-6);
        const double error = setpoint_ - measurement;
        lastError_ = error;

        const double dMeas = isFirstUpdate_ ? 0.0 : (measurement - prevMeas_) / dt;

        if (filterTf_ > 0.0) {
            const double alpha = dt / (filterTf_ + dt);
            derivFilt_ = derivFilt_ + alpha * (dMeas - derivFilt_);
        } else {
            derivFilt_ = dMeas;
        }

        integral_ += error * dt;
        integral_ = clampVal(integral_, -integralMax_, integralMax_);

        const double P = kP_ * error;
        const double I = kI_ * integral_;
        const double D = -kD_ * derivFilt_;
        const double outUnsat = P + I + D + feedForward_;
        const double outSat = clampVal(outUnsat, outputMin_, outputMax_);

        if (kI_ > 0.0) {
            const double aw = (outSat - outUnsat) * (dt / (antiWindupTt_ * kI_));
            integral_ += aw;
            integral_ = clampVal(integral_, -integralMax_, integralMax_);
        }

        prevMeas_ = measurement;
        isFirstUpdate_ = false;
        lastOutput_ = outSat;
        return outSat;
    }

    void reset(double currentMeas = 0.0, bool bumpless = false, double holdOut = 0.0) {
        prevMeas_ = currentMeas;
        derivFilt_ = 0.0;
        isFirstUpdate_ = true;
        lastError_ = 0.0;

        if (bumpless && kI_ > 0.0) {
            const double err = setpoint_ - currentMeas;
            const double targetI = (holdOut - (kP_ * err + feedForward_)) / kI_;
            integral_ = clampVal(targetI, -integralMax_, integralMax_);
        } else {
            integral_ = 0.0;
        }
        lastOutput_ = 0.0;
    }

private:
    double kP_ = 0.0, kI_ = 0.0, kD_ = 0.0;
    double integralMax_ = 1000.0;
    double outputMin_ = -100.0, outputMax_ = 100.0;
    double filterTf_ = 0.0, antiWindupTt_ = 0.2;
    double setpoint_ = 0.0, integral_ = 0.0;
    double prevMeas_ = 0.0, derivFilt_ = 0.0;
    double feedForward_ = 0.0, lastOutput_ = 0.0, lastError_ = 0.0;
    bool isFirstUpdate_ = true;

    static double clampVal(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
};

#endif

