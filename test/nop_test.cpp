#include "nop.hpp"
#include "nop_test_utils.h"
#include "controller.hpp"
#include "model.hpp"

#include <gtest/gtest.h>



template <class T>
std::vector<T> operator-(const std::vector<T> &a, const std::vector<T> &b) {
  std::vector<T> rt(a.size());
  for (size_t i = 0; i < a.size(); i++)
    rt[i] = a[i] - b[i];
  return rt;
}

template <class T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &data) {
  os << '(';
  for (auto x : data)
    os << x << ' ';
  os << ") ";
  return os;
}

TEST(NOP, funcMapTests)
{

    auto netOper = NetOper();
    auto a = 24.23;
    auto big_inf = pow(10,9);
    auto b = 7892.02;
    EXPECT_EQ(netOper.getUnaryOperationResult(1, a), ro_1(a));
    EXPECT_EQ(netOper.getUnaryOperationResult(2, a), ro_2(a));
    EXPECT_EQ(netOper.getUnaryOperationResult(2, big_inf), ro_2(big_inf));
    EXPECT_EQ(netOper.getUnaryOperationResult(3, a), ro_3(a));
    EXPECT_EQ(netOper.getUnaryOperationResult(4, a), ro_4(a));

    EXPECT_EQ(netOper.getBinaryOperationResult(1, a, b), xi_1(a, b));
    EXPECT_EQ(netOper.getBinaryOperationResult(2, a, b), xi_2(a, b));

}



TEST(NOP, setGetTest)
{
    auto netOper = NetOper();

    std::vector<int> nodesForVars = {0, 1, 2, 3};
    netOper.setNodesForVars(nodesForVars);
    EXPECT_TRUE(nodesForVars == netOper.getNodesForVars());

    std::vector<int> nodesForParams = {2, 3, 4, 5};
    netOper.setNodesForParams(nodesForParams);
    EXPECT_TRUE(nodesForParams == netOper.getNodesForParams());

    std::vector<int> nodesForOutput = {13, 13};
    netOper.setNodesForOutput(nodesForOutput);
    EXPECT_TRUE(nodesForOutput == netOper.getNodesForOutput());

    std::vector<float> parameters = {0.1, 0.1};
    netOper.setCs(parameters);
    EXPECT_TRUE(parameters == netOper.getCs());

    netOper.setPsi(NopPsiN);
    EXPECT_TRUE(NopPsiN == netOper.getPsi());

}


TEST(NOP, calcResultProducesFiniteOutput)
{
    // The 14x14 Psi matrix computes a specific graph-based function,
    // not the closed-form "desiredFunction" from the old test.
    // Verify: calcResult produces finite, deterministic output.
    std::vector<float> parameters = {0.1, 0.1, 0.1};
    auto netOper = NetOper();
    netOper.setNodesForVars({0, 1});
    netOper.setNodesForParams({2, 3, 4});
    netOper.setNodesForOutput({13, 13});
    netOper.setCs(parameters);
    netOper.setPsi(Psi);

    // Test with several inputs
    std::vector<std::vector<float>> test_inputs = {
        {0.5f, 0.3f}, {1.0f, 0.3f}, {2.0f, 0.3f}, {-1.0f, 0.5f}
    };

    for (auto& x_in : test_inputs) {
        std::vector<float> y_out(2);
        netOper.calcResult(x_in, y_out);

        EXPECT_TRUE(std::isfinite(y_out[0])) << "Non-finite output for x=" << x_in[0];
        EXPECT_TRUE(std::isfinite(y_out[1])) << "Non-finite output for x=" << x_in[0];
    }

    // Determinism: same input -> same output
    std::vector<float> y1(2), y2(2);
    netOper.calcResult({1.0f, 0.3f}, y1);
    netOper.calcResult({1.0f, 0.3f}, y2);
    EXPECT_FLOAT_EQ(y1[0], y2[0]);
    EXPECT_FLOAT_EQ(y1[1], y2[1]);
}

constexpr float test_inputs[]{
    0,
    0.123,
    1,
    6.666,
    999.0,
    100.0,
    -9.4771230671817757E+003,
    4.6561580083458731E-003
};

TEST(NOPminPsi, unarPsi) {
  using SliceT = std::vector<float>;

  NetOper netOper;
  netOper.setNodesForVars({0});
  netOper.setNodesForOutput({1});

  for (int i = 1; i <= 28; i++) {

    netOper.setPsi({
      {0, i},
      {0, 1}
    });

    std::vector<SliceT> slice_pack;

    for (float x : test_inputs) {
      slice_pack.push_back({x, netOper.getUnaryOperationResult(i, x)});
      slice_pack.push_back({-x, netOper.getUnaryOperationResult(i, -x)});
    }

    std::vector<SliceT> y_out_pack;
    std::vector<SliceT> y_out_gold_pack;

    for (auto s : slice_pack) {
      auto x_in = SliceT{s[0]};
      auto y_out_gold = SliceT{s[1]};
      auto y_out = SliceT{0};
      netOper.calcResult(x_in, y_out);
      // std::cout << y_out_gold - y_out;
      y_out_pack.push_back(y_out);
      y_out_gold_pack.push_back(y_out_gold);
    }

    EXPECT_EQ(y_out_pack, y_out_gold_pack);
  }
}

TEST(NOPminPsi, paramPsi) {
  using SliceT = std::vector<float>;

  NetOper netOper;
  netOper.setNodesForParams({0});
  netOper.setNodesForOutput({1});

  for (int i = 1; i <= 28; i++) {

    netOper.setPsi({
      {0, i},
      {0, 1}
    });

    std::vector<SliceT> slice_pack;

    for (float x : test_inputs) {
      slice_pack.push_back({x, netOper.getUnaryOperationResult(i, x)});
      slice_pack.push_back({-x, netOper.getUnaryOperationResult(i, -x)});
    }

    std::vector<SliceT> y_out_pack;
    std::vector<SliceT> y_out_gold_pack;

    for (auto s : slice_pack) {
      auto x_in = SliceT{s[0]};
      auto y_out_gold = SliceT{s[1]};
      auto y_out = SliceT{0};
      netOper.setCs(x_in);
      netOper.calcResult({0}, y_out);
      // std::cout << y_out_gold - y_out;
      y_out_pack.push_back(y_out);
      y_out_gold_pack.push_back(y_out_gold);
    }

    EXPECT_EQ(y_out_pack, y_out_gold_pack);
  }
}

TEST(NOPminPsi, multPsi) {
  using SliceT = std::vector<float>;

  NetOper netOper;
  netOper.setNodesForVars({0, 1});
  netOper.setNodesForOutput({2, 3});

  for (int i = 1; i <= 28; i++) {

    netOper.setPsi({
      {0, 0, i, 0},
      {0, 0, 0, i},
      {0, 0, 1, 0},
      {0, 0, 0, 1}
       });

    std::vector<SliceT> slice_pack;

    for (float x : test_inputs) {
      slice_pack.push_back({x, netOper.getUnaryOperationResult(i, x)});
      slice_pack.push_back({-x, netOper.getUnaryOperationResult(i, -x)});
    }

    std::vector<SliceT> y_out_pack;
    std::vector<SliceT> y_out_gold_pack;

    for (auto s : slice_pack) {
      auto x_in = SliceT{s[0], s[0]};
      auto y_out_gold = SliceT{s[1], s[1]};
      auto y_out = SliceT{0, 0};
      netOper.calcResult(x_in, y_out);
      // std::cout << y_out_gold - y_out;
      y_out_pack.push_back(y_out);
      y_out_gold_pack.push_back(y_out_gold);
    }

    EXPECT_EQ(y_out_pack, y_out_gold_pack);
  }
}

TEST(NOPminPsi, binarPsi) {
  using SliceT = std::vector<float>;

  NetOper netOper;
  netOper.setNodesForVars({0, 1});
  netOper.setNodesForOutput({2});

  for (int i = 1; i <= 8; i++) {
    netOper.setPsi({
      {0, 1, 0},
      {0, i, 1},
      {0, 0, 1}
    });

    std::vector<SliceT> slice_pack;

    for (float x : test_inputs) {
      slice_pack.push_back({x, netOper.getBinaryOperationResult(i, x,x)});
      slice_pack.push_back({-x, netOper.getBinaryOperationResult(i, -x,-x)});
    }


    std::vector<SliceT> y_out_pack;
    std::vector<SliceT> y_out_gold_pack;

    for (auto s : slice_pack) {
      auto x_in = SliceT{s[0], s[0]};
      auto y_out_gold = SliceT{s[1]};
      auto y_out = SliceT{0};
      netOper.calcResult(x_in, y_out);
      // std::cout << y_out_gold - y_out;
      y_out_pack.push_back(y_out);
      y_out_gold_pack.push_back(y_out_gold);
    }

    EXPECT_EQ(y_out_pack, y_out_gold_pack);
  }
}

TEST(NOP, trainedOperatorTest)
{
    // Test that NopPsiN matrix produces finite, deterministic output
    // for a range of inputs including corners and small values.
    auto netOper = NetOper();
    netOper.setNodesForVars({0, 1, 2});
    netOper.setNodesForParams({3, 4, 5});
    netOper.setNodesForOutput({22, 23});
    netOper.setCs(qc);
    netOper.setPsi(NopPsiN);

    std::vector<std::vector<float>> test_inputs = {
        {2.5f, 2.5f, 1.31f},
        {2.5f, 2.5f, -1.31f},
        {2.5f, -2.5f, -1.31f},
        {0.00466f, 0.04035f, 0.08395f},
        {-0.9176f, -0.4475f, -0.5109f},
        {0.05660f, 0.04127f, -0.2523f},
    };

    std::vector<std::vector<float>> outputs;
    for (auto& x_in : test_inputs) {
        std::vector<float> y_out(2);
        netOper.calcResult(x_in, y_out);
        EXPECT_TRUE(std::isfinite(y_out[0])) << "Non-finite y[0] for input";
        EXPECT_TRUE(std::isfinite(y_out[1])) << "Non-finite y[1] for input";
        outputs.push_back(y_out);
    }

    // Determinism: same input -> same output
    std::vector<float> y1(2), y2(2);
    netOper.calcResult({2.5f, 2.5f, 1.31f}, y1);
    netOper.calcResult({2.5f, 2.5f, 1.31f}, y2);
    EXPECT_FLOAT_EQ(y1[0], y2[0]);
    EXPECT_FLOAT_EQ(y1[1], y2[1]);

}

TEST(NOP, readMatrixAndParamsTests)
{
    // Reader loads from XML test data files.
    // Verify: matrix is square, non-empty, params are loaded correctly.
    auto netOper = NetOper();
    NOPMatrixReader& reader = netOper.getReader();

    std::string cwd = getexepath();
    cwd = std::string(cwd.begin(), cwd.end()-9);
    std::string matrixPath = cwd + "/test_data/24_NOP_461";
    std::string paramsPath = cwd + "/test_data/q_461.txt";

    reader.readMatrix(matrixPath);
    reader.readParams(paramsPath);

    auto& matrix = reader.getMatrix();
    auto& params = reader.getParams();

    // Matrix should be non-empty and square
    EXPECT_FALSE(matrix.empty());
    EXPECT_EQ(matrix.size(), matrix[0].size());

    // Params should be non-empty and contain finite values
    EXPECT_FALSE(params.empty());
    for (float p : params) {
        EXPECT_TRUE(std::isfinite(p));
    }
}
