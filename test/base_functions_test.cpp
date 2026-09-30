#include "baseFunctions.hpp"
#include <gtest/gtest.h>
#include <cmath>

// Test ro_1: identity function
TEST(BaseFunctions, ro_1_identity) {
    EXPECT_FLOAT_EQ(ro_1(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_1(5.0f), 5.0f);
    EXPECT_FLOAT_EQ(ro_1(-3.14f), -3.14f);
    EXPECT_FLOAT_EQ(ro_1(1e6f), 1e6f);
}

// Test ro_2: square function with infinity protection
TEST(BaseFunctions, ro_2_square) {
    EXPECT_FLOAT_EQ(ro_2(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_2(3.0f), 9.0f);
    EXPECT_FLOAT_EQ(ro_2(-4.0f), 16.0f);
    // Test infinity protection
    float big = sqrt(Infinity) + 1.0f;
    EXPECT_FLOAT_EQ(ro_2(big), Infinity);
}

// Test ro_3: negation
TEST(BaseFunctions, ro_3_negation) {
    EXPECT_FLOAT_EQ(ro_3(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_3(5.0f), -5.0f);
    EXPECT_FLOAT_EQ(ro_3(-3.0f), 3.0f);
}

// Test ro_4: sign * sqrt(|x|)
TEST(BaseFunctions, ro_4_sign_sqrt_abs) {
    EXPECT_FLOAT_EQ(ro_4(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_4(4.0f), 2.0f);    // sqrt(4) = 2
    EXPECT_FLOAT_EQ(ro_4(-4.0f), -2.0f);  // -sqrt(4) = -2
    EXPECT_NEAR(ro_4(2.0f), 1.4142f, 0.0001f);
}

// Test ro_5: reciprocal with epsilon protection
TEST(BaseFunctions, ro_5_reciprocal) {
    EXPECT_FLOAT_EQ(ro_5(2.0f), 0.5f);
    EXPECT_FLOAT_EQ(ro_5(-4.0f), -0.25f);
    // Test epsilon protection for small values
    EXPECT_FLOAT_EQ(ro_5(0.0f), 1.0f / Eps);
    EXPECT_FLOAT_EQ(ro_5(Eps / 2.0f), 1.0f / Eps);
}

// Test ro_6: exponential with max protection
TEST(BaseFunctions, ro_6_exp) {
    EXPECT_FLOAT_EQ(ro_6(0.0f), 1.0f);
    EXPECT_NEAR(ro_6(1.0f), 2.71828f, 0.0001f);
    // Test max protection
    EXPECT_FLOAT_EQ(ro_6(-logf(Eps) + 1.0f), -logf(Eps));
}

// Test ro_7: log with epsilon protection
TEST(BaseFunctions, ro_7_log) {
  // Normal values: log(|x|)
  EXPECT_FLOAT_EQ(ro_7(1.0f), 0.0f);                    // log(1) = 0
  EXPECT_NEAR(ro_7(expf(1.0f)), 1.0f, 0.0001f);         // log(e) = 1
  EXPECT_NEAR(ro_7(100.0f), log(100.0f), 0.0001f);

  // Very small values: epsilon protection
  float tiny = expf(-PokMax) / 2.0f;
  EXPECT_FLOAT_EQ(ro_7(tiny), log(Eps));

  // Zero: epsilon protection
  EXPECT_FLOAT_EQ(ro_7(0.0f), log(Eps));
}

// Test ro_8: sigmoid-like function
TEST(BaseFunctions, ro_8_sigmoid) {
    EXPECT_FLOAT_EQ(ro_8(0.0f), 0.0f);
    EXPECT_GT(ro_8(1.0f), 0.0f);
    EXPECT_LT(ro_8(-1.0f), 0.0f);
    // Test infinity protection
    float big = -log(Eps) + 1.0f;
    EXPECT_FLOAT_EQ(ro_8(big), 1.0f);
}

// Test ro_9: step function (>= 0 -> 1, < 0 -> 0)
TEST(BaseFunctions, ro_9_step) {
    EXPECT_FLOAT_EQ(ro_9(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_9(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_9(-0.1f), 0.0f);
    EXPECT_FLOAT_EQ(ro_9(-5.0f), 0.0f);
}

// Test ro_10: sign function
TEST(BaseFunctions, ro_10_sign) {
    EXPECT_FLOAT_EQ(ro_10(5.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_10(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_10(-5.0f), -1.0f);
}

// Test ro_11: cosine
TEST(BaseFunctions, ro_11_cos) {
    EXPECT_FLOAT_EQ(ro_11(0.0f), 1.0f);
    EXPECT_NEAR(ro_11(M_PI), -1.0f, 0.0001f);
    EXPECT_NEAR(ro_11(M_PI / 2.0f), 0.0f, 0.0001f);
}

// Test ro_12: sine
TEST(BaseFunctions, ro_12_sin) {
    EXPECT_FLOAT_EQ(ro_12(0.0f), 0.0f);
    EXPECT_NEAR(ro_12(M_PI / 2.0f), 1.0f, 0.0001f);
    EXPECT_NEAR(ro_12(M_PI), 0.0f, 0.0001f);
}

// Test ro_13: arctangent
TEST(BaseFunctions, ro_13_atan) {
    EXPECT_FLOAT_EQ(ro_13(0.0f), 0.0f);
    EXPECT_NEAR(ro_13(1.0f), M_PI / 4.0f, 0.0001f);
    EXPECT_NEAR(ro_13(-1.0f), -M_PI / 4.0f, 0.0001f);
}

// Test ro_14: cube with infinity protection
TEST(BaseFunctions, ro_14_cube) {
    EXPECT_FLOAT_EQ(ro_14(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_14(2.0f), 8.0f);
    EXPECT_FLOAT_EQ(ro_14(-2.0f), -8.0f);
    // Test infinity protection
    float big = cbrtf(Infinity) + 1.0f;
    EXPECT_FLOAT_EQ(ro_14(big), Infinity);
}

// Test ro_15: cube root with epsilon protection
TEST(BaseFunctions, ro_15_cbrt) {
  // ro_15 returns: sign(x) * exp(log(|x|)/3) = sign(x) * |x|^(1/3)
  // For x=0, the function returns sign(0)*Eps = 1*Eps (since ro_10(0)=1)
  EXPECT_FLOAT_EQ(ro_15(8.0f), 2.0f);
  EXPECT_FLOAT_EQ(ro_15(-8.0f), -2.0f);
  // Test epsilon protection for small values
  EXPECT_FLOAT_EQ(ro_15(Eps / 2.0f), Eps);  // Returns Eps for very small values
}

// Test ro_16: saturation (|x| < 1 -> x, else sign(x))
TEST(BaseFunctions, ro_16_saturation) {
    EXPECT_FLOAT_EQ(ro_16(0.5f), 0.5f);
    EXPECT_FLOAT_EQ(ro_16(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_16(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_16(-0.5f), -0.5f);
    EXPECT_FLOAT_EQ(ro_16(-1.0f), -1.0f);
    EXPECT_FLOAT_EQ(ro_16(2.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_16(-2.0f), -1.0f);
}

// Test ro_17: sign(x) * log(|x| + 1)
TEST(BaseFunctions, ro_17_signed_log) {
    EXPECT_FLOAT_EQ(ro_17(0.0f), 0.0f);
    EXPECT_NEAR(ro_17(1.0f), log(2.0f), 0.0001f);
    EXPECT_NEAR(ro_17(-1.0f), -log(2.0f), 0.0001f);
}

// Test ro_18: sign(x) * (exp(|x|) - 1) with infinity protection
TEST(BaseFunctions, ro_18_exp_abs_minus_one) {
    EXPECT_FLOAT_EQ(ro_18(0.0f), 0.0f);
    EXPECT_NEAR(ro_18(1.0f), exp(1.0f) - 1.0f, 0.0001f);
    EXPECT_NEAR(ro_18(-1.0f), -(exp(1.0f) - 1.0f), 0.0001f);
    // Test infinity protection
    float big = -log(Eps) + 1.0f;
    EXPECT_FLOAT_EQ(ro_18(big), Infinity);
}

// Test ro_19: sign(x) * exp(-|x|)
TEST(BaseFunctions, ro_19_signed_exp_neg_abs) {
  // ro_19 returns sign(x) * exp(-|x|), which at x=0 gives sign(0)*exp(0) = 1*1 = 1
  EXPECT_FLOAT_EQ(ro_19(0.0f), 1.0f);  // ro_10(0) returns 1, exp(0) = 1
  EXPECT_NEAR(ro_19(1.0f), exp(-1.0f), 0.0001f);
  EXPECT_NEAR(ro_19(-1.0f), -exp(-1.0f), 0.0001f);
  // Test large value -> approaches 0
  EXPECT_NEAR(ro_19(10.0f), 0.0f, 0.0001f);
}

// Test ro_20: x / 2
TEST(BaseFunctions, ro_20_divide_by_2) {
    EXPECT_FLOAT_EQ(ro_20(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_20(4.0f), 2.0f);
    EXPECT_FLOAT_EQ(ro_20(-6.0f), -3.0f);
}

// Test ro_21: x * 2
TEST(BaseFunctions, ro_21_multiply_by_2) {
    EXPECT_FLOAT_EQ(ro_21(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_21(3.0f), 6.0f);
    EXPECT_FLOAT_EQ(ro_21(-2.5f), -5.0f);
}

// Test ro_22: exponential-like function
TEST(BaseFunctions, ro_22_exp_like) {
    EXPECT_FLOAT_EQ(ro_22(0.0f), 0.0f);
    EXPECT_NEAR(ro_22(1.0f), 1.0f - exp(-1.0f), 0.0001f);
    EXPECT_NEAR(ro_22(-1.0f), exp(-1.0f) - 1.0f, 0.0001f);
}

// Test ro_23: x - x^3
TEST(BaseFunctions, ro_23_cubic) {
    EXPECT_FLOAT_EQ(ro_23(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_23(1.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_23(2.0f), -6.0f);
    EXPECT_FLOAT_EQ(ro_23(-2.0f), 6.0f);
}

// Test ro_24: sigmoid function with infinity protection
TEST(BaseFunctions, ro_24_sigmoid) {
    EXPECT_FLOAT_EQ(ro_24(0.0f), 0.5f);
    EXPECT_GT(ro_24(1.0f), 0.5f);
    EXPECT_LT(ro_24(-1.0f), 0.5f);
    EXPECT_NEAR(ro_24(10.0f), 1.0f, 0.0001f);
    EXPECT_NEAR(ro_24(-10.0f), 0.0f, 0.0001f);
}

// Test ro_25: Heaviside step (> 0 -> 1, <= 0 -> 0)
TEST(BaseFunctions, ro_25_heaviside) {
    EXPECT_FLOAT_EQ(ro_25(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_25(0.1f), 1.0f);
    EXPECT_FLOAT_EQ(ro_25(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_25(-0.1f), 0.0f);
    EXPECT_FLOAT_EQ(ro_25(-5.0f), 0.0f);
}

// Test ro_26: sign(x) with epsilon protection
TEST(BaseFunctions, ro_26_sign_epsilon) {
    EXPECT_FLOAT_EQ(ro_26(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_26(-1.0f), -1.0f);
    EXPECT_FLOAT_EQ(ro_26(0.005f), 0.0f);  // |x| < 0.01
    EXPECT_FLOAT_EQ(ro_26(-0.005f), 0.0f);
    EXPECT_FLOAT_EQ(ro_26(0.02f), 1.0f);   // |x| >= 0.01
}

// Test ro_27: sign(x) * (1 - sqrt(1 - x^2)) for |x| <= 1
TEST(BaseFunctions, ro_27_circular) {
    EXPECT_FLOAT_EQ(ro_27(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ro_27(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_27(-1.0f), -1.0f);
    EXPECT_FLOAT_EQ(ro_27(2.0f), 1.0f);
    EXPECT_FLOAT_EQ(ro_27(-2.0f), -1.0f);
}

// Test ro_28: x * (1 - exp(-x^2))
TEST(BaseFunctions, ro_28_gaussian_like) {
    EXPECT_FLOAT_EQ(ro_28(0.0f), 0.0f);
    EXPECT_NEAR(ro_28(1.0f), 1.0f * (1.0f - exp(-1.0f)), 0.0001f);
    EXPECT_NEAR(ro_28(-1.0f), -1.0f * (1.0f - exp(-1.0f)), 0.0001f);
}

// Test xi_1: addition
TEST(BaseFunctions, xi_1_addition) {
    EXPECT_FLOAT_EQ(xi_1(3.0f, 4.0f), 7.0f);
    EXPECT_FLOAT_EQ(xi_1(-2.0f, 5.0f), 3.0f);
    EXPECT_FLOAT_EQ(xi_1(0.0f, 0.0f), 0.0f);
}

// Test xi_2: multiplication with infinity protection
TEST(BaseFunctions, xi_2_multiplication) {
    EXPECT_FLOAT_EQ(xi_2(3.0f, 4.0f), 12.0f);
    EXPECT_FLOAT_EQ(xi_2(-2.0f, 5.0f), -10.0f);
    EXPECT_FLOAT_EQ(xi_2(0.0f, 100.0f), 0.0f);
    // Test infinity protection
    EXPECT_FLOAT_EQ(xi_2(sqrt(Infinity) + 1.0f, sqrt(Infinity) + 1.0f), Infinity);
}

// Test xi_3: maximum
TEST(BaseFunctions, xi_3_maximum) {
    EXPECT_FLOAT_EQ(xi_3(3.0f, 4.0f), 4.0f);
    EXPECT_FLOAT_EQ(xi_3(5.0f, 2.0f), 5.0f);
    EXPECT_FLOAT_EQ(xi_3(3.0f, 3.0f), 3.0f);
}

// Test xi_4: minimum
TEST(BaseFunctions, xi_4_minimum) {
    EXPECT_FLOAT_EQ(xi_4(3.0f, 4.0f), 3.0f);
    EXPECT_FLOAT_EQ(xi_4(5.0f, 2.0f), 2.0f);
    EXPECT_FLOAT_EQ(xi_4(3.0f, 3.0f), 3.0f);
}

// Test xi_5: fuzzy OR (x + y - x*y)
TEST(BaseFunctions, xi_5_fuzzy_or) {
    EXPECT_FLOAT_EQ(xi_5(0.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(xi_5(1.0f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(xi_5(0.0f, 1.0f), 1.0f);
    EXPECT_FLOAT_EQ(xi_5(1.0f, 1.0f), 1.0f);
    EXPECT_FLOAT_EQ(xi_5(0.5f, 0.5f), 0.75f);
}

// Test xi_6: sign(x + y) * sqrt(x^2 + y^2)
TEST(BaseFunctions, xi_6_euclidean) {
    EXPECT_FLOAT_EQ(xi_6(3.0f, 4.0f), 5.0f);  // sqrt(9 + 16) = 5
    EXPECT_FLOAT_EQ(xi_6(-3.0f, -4.0f), -5.0f);
    EXPECT_FLOAT_EQ(xi_6(0.0f, 0.0f), 0.0f);
    EXPECT_NEAR(xi_6(1.0f, 1.0f), sqrt(2.0f), 0.0001f);
}

// Test xi_7: sign(x + y) * (|x| + |y|)
TEST(BaseFunctions, xi_7_manhattan) {
  EXPECT_FLOAT_EQ(xi_7(3.0f, 4.0f), 7.0f);
  EXPECT_FLOAT_EQ(xi_7(-3.0f, -4.0f), -7.0f);
  // When x+y > 0, sign is positive: |x| + |y|
  // -3 + 4 = 1 > 0, so sign is +1, result is |-3| + |4| = 3 + 4 = 7
  EXPECT_FLOAT_EQ(xi_7(-3.0f, 4.0f), 7.0f);
  EXPECT_FLOAT_EQ(xi_7(0.0f, 0.0f), 0.0f);
}

// Test xi_8: sign(x + y) * |x| * |y|
TEST(BaseFunctions, xi_8_signed_product) {
  EXPECT_FLOAT_EQ(xi_8(3.0f, 4.0f), 12.0f);
  EXPECT_FLOAT_EQ(xi_8(-3.0f, -4.0f), -12.0f);
  // When x+y > 0, sign is positive: |x| * |y|
  // -3 + 4 = 1 > 0, so sign is +1, result is |-3| * |4| = 3 * 4 = 12
  EXPECT_FLOAT_EQ(xi_8(-3.0f, 4.0f), 12.0f);
  EXPECT_FLOAT_EQ(xi_8(0.0f, 5.0f), 0.0f);
}

// Test all functions with typical values
TEST(BaseFunctions, all_functions_consistency) {
    // Test that all functions are defined and callable
    float test_val = 0.5f;
    
    // Unary functions
    EXPECT_NO_THROW(ro_1(test_val));
    EXPECT_NO_THROW(ro_2(test_val));
    EXPECT_NO_THROW(ro_3(test_val));
    EXPECT_NO_THROW(ro_4(test_val));
    EXPECT_NO_THROW(ro_5(test_val));
    EXPECT_NO_THROW(ro_6(test_val));
    EXPECT_NO_THROW(ro_7(test_val));
    EXPECT_NO_THROW(ro_8(test_val));
    EXPECT_NO_THROW(ro_9(test_val));
    EXPECT_NO_THROW(ro_10(test_val));
    EXPECT_NO_THROW(ro_11(test_val));
    EXPECT_NO_THROW(ro_12(test_val));
    EXPECT_NO_THROW(ro_13(test_val));
    EXPECT_NO_THROW(ro_14(test_val));
    EXPECT_NO_THROW(ro_15(test_val));
    EXPECT_NO_THROW(ro_16(test_val));
    EXPECT_NO_THROW(ro_17(test_val));
    EXPECT_NO_THROW(ro_18(test_val));
    EXPECT_NO_THROW(ro_19(test_val));
    EXPECT_NO_THROW(ro_20(test_val));
    EXPECT_NO_THROW(ro_21(test_val));
    EXPECT_NO_THROW(ro_22(test_val));
    EXPECT_NO_THROW(ro_23(test_val));
    EXPECT_NO_THROW(ro_24(test_val));
    EXPECT_NO_THROW(ro_25(test_val));
    EXPECT_NO_THROW(ro_26(test_val));
    EXPECT_NO_THROW(ro_27(test_val));
    EXPECT_NO_THROW(ro_28(test_val));
    
    // Binary functions
    EXPECT_NO_THROW(xi_1(test_val, test_val));
    EXPECT_NO_THROW(xi_2(test_val, test_val));
    EXPECT_NO_THROW(xi_3(test_val, test_val));
    EXPECT_NO_THROW(xi_4(test_val, test_val));
    EXPECT_NO_THROW(xi_5(test_val, test_val));
    EXPECT_NO_THROW(xi_6(test_val, test_val));
    EXPECT_NO_THROW(xi_7(test_val, test_val));
    EXPECT_NO_THROW(xi_8(test_val, test_val));
}
