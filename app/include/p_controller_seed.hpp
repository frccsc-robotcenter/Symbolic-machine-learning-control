/**
 * @file p_controller_seed.hpp
 * @brief П-регулятор go-to-goal, выраженный как NetOper.
 *
 * Семантика выходов совпадает с ROS-нодой и ONNX-моделью:
 *   u_left  ≈ v_cmd  (линейная скорость)
 *   u_right ≈ w_cmd  (угловая скорость)
 *
 * Входы (nodes_for_vars): 0=dist, 1=bearing_err, 2=yaw_err
 * Параметры (nodes_for_params): 3=kv, 4=kw (остальные 0)
 *
 *   u_left  = kv * dist * cos(bearing_err)   // не ехать боком
 *   u_right = kw * bearing_err
 */
#pragma once

#include <vector>
#include <cmath>

namespace p_seed {

constexpr int kMatrixSize = 24;
constexpr float kDefaultKv = 0.5f;
constexpr float kDefaultKw = 1.0f;

/// 24x24 матрица П-регулятора (v_cmd, w_cmd)
inline std::vector<std::vector<int>> makePControllerMatrix()
{
    std::vector<std::vector<int>> m(kMatrixSize, std::vector<int>(kMatrixSize, 0));

    // z20 = kv * dist   (diag 2 = произведение, init 1)
    m[20][20] = 2;
    m[0][20] = 1;   // * dist
    m[3][20] = 1;   // * kv

    // z22 = u_left = z20 * cos(bearing_err)
    //   i=1: z22 = cos(bearing)   (ro_11)
    //   i=20: z22 = cos(bearing) * z20
    m[22][22] = 2;
    m[1][22] = 11;  // ro_11 = cos
    m[20][22] = 1;

    // z23 = u_right = kw * bearing_err
    m[23][23] = 2;
    m[1][23] = 1;
    m[4][23] = 1;

    return m;
}

/// 8 параметров: kv, kw, 0...
inline std::vector<float> makePControllerParams(float kv = kDefaultKv, float kw = kDefaultKw)
{
    return {kv, kw, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

} // namespace p_seed
