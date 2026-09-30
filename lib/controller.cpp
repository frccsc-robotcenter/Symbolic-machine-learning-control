#include "controller.hpp"

Controller::Controller(const Model::State &goalState, NetOper &netOper):
	m_goal(goalState),
	m_netOper(netOper)
	{ }

float Controller::wrapAngle(float a)
{
	return std::atan2(std::sin(a), std::cos(a));
}

void Controller::computeFeatures(const Model::State &goal, const Model::State &curr,
                                 float &dist, float &bearing_err, float &yaw_err)
{
	const float dx = goal.x - curr.x;
	const float dy = goal.y - curr.y;
	dist = std::sqrt(dx * dx + dy * dy);
	bearing_err = wrapAngle(std::atan2(dy, dx) - curr.yaw);
	yaw_err = wrapAngle(goal.yaw - curr.yaw);
}

Model::Control Controller::calcControl(const Model::State& currState)
{
	float dist = 0.0f, bearing_err = 0.0f, yaw_err = 0.0f;
	computeFeatures(m_goal, currState, dist, bearing_err, yaw_err);

	std::vector<float> u(2, 0);
	m_netOper.calcResult({dist, bearing_err, yaw_err}, u);
	// Clamp to [-Umax, Umax], handle NaN (replace with 0)
	u[0] = std::isfinite(u[0]) ? std::min(std::max(u[0], -Umax), Umax) : 0.0f;
	u[1] = std::isfinite(u[1]) ? std::min(std::max(u[1], -Umax), Umax) : 0.0f;

	return Model::Control{u[0], u[1]};
}

void Controller::setGoal(Model::State newGoal)
{
	if(m_goal == newGoal) return;
	m_goal = newGoal;
}

NetOper& Controller::netOper()
{
	return m_netOper;
}

void Controller::setUMax(float newUMax)
{
	if(newUMax == Umax) return;
	Umax = newUMax;
}
