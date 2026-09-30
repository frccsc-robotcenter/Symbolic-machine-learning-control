#include "model.hpp"

// Model::Control

Model::Control Model::Control::operator+(const Model::Control &ctrl)
{
  return Model::Control{this->left + ctrl.left, this->right + ctrl.right};
}

Model::Control Model::Control::operator-(const Model::Control &ctrl) 
{
  return Model::Control{this->left - ctrl.left, this->right - ctrl.right};
}

Model::Control Model::Control::operator*(float val) 
{
  return Control{this->left * val, this->right * val};
}

// Model::State

bool Model::State::operator==(const Model::State &state) const
{
  return (this->x == state.x) && (this->y == state.y) && (this->yaw == state.yaw);
}

Model::State Model::State::operator+(const Model::State &state) const
{
  return Model::State{this->x + state.x, this->y + state.y, this->yaw + state.yaw};
}

Model::State Model::State::operator-(const Model::State &state) const
{
  const float &yaw1=this->yaw;
  const float &yaw2=state.yaw;
  float dyaw = atan2f(sinf(yaw1 - yaw2), cosf(yaw1 - yaw2));
  return State{this->x - state.x, this->y - state.y, dyaw};
}

Model::State Model::State::operator*(float val) const
{
  return Model::State{this->x * val, this->y * val, this->yaw * val};
}

float Model::State::dist(const Model::State &state) const
{
  float dx = fabs(this->x - state.x);
  float dy = fabs(this->y - state.y);
  float dyaw = fabs(this->yaw - state.yaw);

  return std::sqrt(dx * dx + dy * dy + dyaw * dyaw);
}

float Model::State::distXY(const Model::State &state) const
{
  auto ds = this->operator-(state);
  return std::sqrt(ds.x * ds.x + ds.y * ds.y);
}

const void Model::State::print() const
{
  std::cout << x << " " << y << " " << yaw << "\n";
}

// Model::Model

Model::Model(const State &state, float dt, const std::string &onnx_path)
      : m_currentState(state),
        m_dt(dt),
        m_env(ORT_LOGGING_LEVEL_WARNING, "RobotNN"),
        m_session(m_env, onnx_path.c_str(), Ort::SessionOptions{}) {
    // Cache input/output names once
    Ort::AllocatorWithDefaultOptions alloc;
    Ort::AllocatedStringPtr in = m_session.GetInputNameAllocated(0, alloc);
    m_input_name = in.get();
    Ort::AllocatedStringPtr out = m_session.GetOutputNameAllocated(0, alloc);
    m_output_name = out.get();
}

void Model::setState(const Model::State &state) 
{ 
  m_currentState = state; 
}

void Model::setVelocity(const float new_v, const float new_w) 
{ 
  m_v = new_v;
  m_w = new_w;
}


const Model::State& Model::getState() 
{ 
  return m_currentState; 
}

Model::State Model::velocityFromControl(const Model::Control &u) 
{
  return  State{k * (u.left + u.right) * cosf(m_currentState.yaw),
                k * (u.left + u.right) * sinf(m_currentState.yaw),
                k_w * k * (u.left - u.right)};
}

Model::State Model::nextStateFromVelocity(Model::State &vel) 
{
  return m_currentState + vel * m_dt;
}

Model::State Model::nextStateFromControl(const Model::Control &u) 
{
  Model::State vel = velocityFromControl(u);
  return nextStateFromVelocity(vel);
}


Model::State Model::nextNNStateFromControl(const Model::Control &u) {
    std::array<float, 5> input_vals = {m_v, m_w, u.left, u.right, m_dt};
    std::array<int64_t, 2> dims{1, 5};

    Ort::MemoryInfo mem_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem_info, input_vals.data(), input_vals.size(), dims.data(), dims.size());

    const char* in_name = m_input_name.c_str();
    const char* out_name = m_output_name.c_str();

    auto outputs = m_session.Run(Ort::RunOptions{nullptr},
                                 &in_name, &input_tensor, 1,
                                 &out_name, 1);

    float *out_data = outputs[0].GetTensorMutableData<float>();
    m_v = out_data[0];
    m_w = out_data[1];

    auto vel = State{m_v * cosf(m_currentState.yaw),
                     m_v * sinf(m_currentState.yaw),
                     m_w};

    return nextStateFromVelocity(vel);
}

