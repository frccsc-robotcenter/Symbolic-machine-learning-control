#include "controller.hpp"
#include "p_controller_seed.hpp"

#include <gtest/gtest.h>


TEST(Controller, ConstructorTests)
{
    NetOper newNetOper = NetOper();
    Model::State goal = {1., 1., 0.};
    Controller second = Controller(goal, newNetOper);
}

TEST(Controller, WrapAngle)
{
    EXPECT_NEAR(Controller::wrapAngle(0.0f), 0.0f, 1e-5f);
    // ±pi: atan2 folds to (-pi, pi]; accept either sign at the cut
    float w_pi = Controller::wrapAngle(3.14159265f);
    EXPECT_NEAR(std::fabs(w_pi), 3.14159265f, 1e-4f);
    float w_mpi = Controller::wrapAngle(-3.14159265f);
    EXPECT_NEAR(std::fabs(w_mpi), 3.14159265f, 1e-4f);
    EXPECT_NEAR(Controller::wrapAngle(2.0f * 3.14159265f), 0.0f, 1e-4f);
    EXPECT_NEAR(Controller::wrapAngle(3.14159265f + 0.5f), -3.14159265f + 0.5f, 1e-4f);
}

TEST(Controller, ComputeFeatures)
{
    Model::State goal = {2.0f, 0.0f, 0.0f};
    Model::State curr = {0.0f, 0.0f, 0.0f};

    float dist, bearing, yaw_err;
    Controller::computeFeatures(goal, curr, dist, bearing, yaw_err);

    EXPECT_NEAR(dist, 2.0f, 1e-5f);
    EXPECT_NEAR(bearing, 0.0f, 1e-5f);   // goal straight ahead
    EXPECT_NEAR(yaw_err, 0.0f, 1e-5f);

    // Goal to the left (+y), robot facing +x
    goal = {0.0f, 3.0f, 0.0f};
    Controller::computeFeatures(goal, curr, dist, bearing, yaw_err);
    EXPECT_NEAR(dist, 3.0f, 1e-5f);
    EXPECT_NEAR(bearing, 3.14159265f / 2.0f, 1e-4f);  // +90 deg left
}

TEST(Controller, PControllerSigns)
{
    NetOper net;
    net.setNodesForVars({0, 1, 2});
    net.setNodesForParams({3, 4, 5, 6, 7, 8, 9, 10});
    net.setNodesForOutput({22, 23});
    net.setPsi(p_seed::makePControllerMatrix());
    net.setCs(p_seed::makePControllerParams(0.5f, 1.0f));

    Model::State goal = {0.0f, 0.0f, 0.0f};
    Controller controller(goal, net);
    controller.setUMax(1.0f);

    // Goal straight ahead (robot at (-2,0,0) → bearing=0)
    // u_left=v_cmd > 0, u_right=w_cmd ≈ 0
    Model::State curr = {-2.0f, 0.0f, 0.0f};
    Model::Control u = controller.calcControl(curr);
    EXPECT_TRUE(std::isfinite(u.left));
    EXPECT_TRUE(std::isfinite(u.right));
    EXPECT_GT(u.left, 0.0f);   // v_cmd forward
    EXPECT_NEAR(u.right, 0.0f, 1e-3f);  // no turn

    // Goal to the left (bearing=+pi/2) → w_cmd > 0 (turn left)
    controller.setGoal({0.0f, 2.0f, 0.0f});
    curr = {0.0f, 0.0f, 0.0f};
    u = controller.calcControl(curr);
    EXPECT_GT(u.right, 0.0f);
    // cos(pi/2)=0 → v_cmd ≈ 0 when goal is fully to the side
    EXPECT_NEAR(u.left, 0.0f, 1e-3f);

    // Goal to the right (bearing=-pi/2) → w_cmd < 0
    controller.setGoal({0.0f, -2.0f, 0.0f});
    u = controller.calcControl(curr);
    EXPECT_LT(u.right, 0.0f);

    // Outputs clamped
    controller.setGoal({10.0f, 10.0f, 0.0f});
    curr = {-10.0f, -10.0f, 0.0f};
    u = controller.calcControl(curr);
    EXPECT_LE(u.left, 1.0f);
    EXPECT_GE(u.left, -1.0f);
    EXPECT_LE(u.right, 1.0f);
    EXPECT_GE(u.right, -1.0f);
}

TEST(Controller, SimpleTest)
{
    NetOper newNetOper = NetOper();
    Model::State goal = {0.1, 0.2, 0.};
    Controller controller = Controller(goal, newNetOper);

    std::vector<float> parameters = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    controller.netOper().setNodesForVars({0, 1, 2});
    controller.netOper().setNodesForParams({3, 4, 5, 6, 7, 8, 9, 10});
    controller.netOper().setNodesForOutput({22, 23});
    controller.netOper().setCs(parameters);
    controller.netOper().setPsi(p_seed::makePControllerMatrix());

    Model::State currState = {0, 0, 0};
    Model::Control u = controller.calcControl(currState);

    // Output should be finite and within Umax bounds
    EXPECT_TRUE(std::isfinite(u.left));
    EXPECT_TRUE(std::isfinite(u.right));
    EXPECT_LE(u.left, 1.0f);   // Umax = 1.0
    EXPECT_GE(u.left, -1.0f);
    EXPECT_LE(u.right, 1.0f);
    EXPECT_GE(u.right, -1.0f);
}
