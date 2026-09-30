#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <onnxruntime_cxx_api.h>

class Model {

public:

struct Control
{
  float left;
  float right;

  Control operator+(const Control &ctrl);
  Control operator-(const Control &ctrl);
  Control operator*(float val);
};

struct State
{
  float x;
  float y;
  float yaw;

  bool operator==(const State &state) const;
  State operator+(const State &state) const;
  State operator-(const State &state) const;
  State operator*(float val) const;
  float dist(const State &state) const;
  float distXY(const State &state) const;
  const void print() const;
};

public:
  Model(const State &state, float dt, const std::string &onnx_path);
  void setState(const State &state);
  void setVelocity(const float new_v, const float new_w);
  const State& getState();
  State velocityFromControl(const Control &u);
  State nextStateFromVelocity(State &vel);
  State nextStateFromControl(const Control &u);
  State nextNNStateFromControl(const Control &u);

  float m_v = 0.0f, m_w = 0.0f;

private:
  float k_w = 1.0f;
  float k = 1.0f;
  State m_currentState;
  float m_dt;

  // onnx runtime
  Ort::Env m_env;
  Ort::Session m_session;

  // Cached ONNX input/output names (avoid per-call string allocation)
  std::string m_input_name;
  std::string m_output_name;
};
