#pragma once

#include "model.hpp"
#include "nop.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

class Controller
{
public:
  Controller(const Model::State &goalState, NetOper &netOper);
  /// RP from pascal version
  virtual Model::Control calcControl(const Model::State &currState);
  /// set new goal state
  void setGoal(Model::State newGoal);

  NetOper& netOper();

  void setUMax(float newUMax);

  /// Wrap angle to (-pi, pi]
  static float wrapAngle(float a);

  /// Features fed into NetOper: {dist, bearing_err, yaw_err}
  static void computeFeatures(const Model::State &goal, const Model::State &curr,
                              float &dist, float &bearing_err, float &yaw_err);

protected:
  Model::State m_goal;
  NetOper& m_netOper;
  float Umax = 1.0f;
};
