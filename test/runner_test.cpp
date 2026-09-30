#include "runner.hpp"

#include <gtest/gtest.h>

TEST(Runner, FullTest)
{
    // This test requires the ONNX model file. Skip if not found.
    std::string onnx_path = "../rosbot_gazebo9_2d_model.onnx";
    {
        std::ifstream f(onnx_path);
        if (!f.good()) {
            GTEST_SKIP() << "ONNX model not found at " << onnx_path
                         << ". Run from build/ with model in project root.";
        }
    }

    NetOper netOp = NetOper();
    netOp.setNodesForVars({0, 1, 2});
    netOp.setNodesForParams({3, 4, 5});
    netOp.setNodesForOutput({22, 23});
    netOp.setCs(qc);
    netOp.setPsi(NopPsiN);

    constexpr float dt = 0.01;
    Model::State currState = {0.0, 0.0, 0.0};
    Model model(currState, dt, onnx_path);

    Model::State goal = {0.0, 0.0, 0.0};
    Controller controller(goal, netOp);
    Runner runner(model, controller);
    runner.setGoal(goal);

    std::vector<Model::State> init_states;
    std::vector<float> qyminc = {-2.5, -2.5, -1.31};
    std::vector<float> qymaxc = {2.5, 2.5, 1.31};

    for (int i = 0; i < 8; ++i) {
        init_states.push_back(Model::State{
            i & 4 ? qymaxc[0] : qyminc[0],
            i & 2 ? qymaxc[1] : qyminc[1],
            i & 1 ? qymaxc[2] : qyminc[2]
        });
    }

    float timeLimit = 1.5;
    float epsterm = 0.1;
    float sumt = 0.0;
    float sumdelt = 0.0;

    for (int i = 0; i < 8; ++i) {
        runner.init(init_states[i]);
        float currTime = 0;
        while (currTime < timeLimit) {
            currState = runner.makeStep();
            currTime += dt;
            if (currState.dist(goal) < epsterm)
                break;
        }
        sumt += currTime;
        sumdelt += currState.dist(goal);
    }

    // Verify simulation ran and produced reasonable results
    EXPECT_GT(sumt, 0.0f);
    EXPECT_TRUE(std::isfinite(sumt));
    EXPECT_TRUE(std::isfinite(sumdelt));
    EXPECT_GE(sumdelt, 0.0f);
}
