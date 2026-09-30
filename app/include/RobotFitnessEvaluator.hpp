/**
 * @file RobotFitnessEvaluator.hpp
 * @brief Evaluator для задачи робота: shaping-фитнес против орбит и залипаний
 */

#pragma once
#include "base_evaluator.hpp"
#include "RobotProblemConfig.hpp"
#include "controller.hpp"
#include "runner.hpp"
#include "model.hpp"
#include <vector>
#include <cmath>
#include <algorithm>


class RobotFitnessEvaluator : public BaseFitnessEvaluator {
public:
    explicit RobotFitnessEvaluator(const RobotProblemConfig& config, int num_objectives = 1)
        : BaseFitnessEvaluator(num_objectives), config_(config),
          model_({0.0f, 0.0f, 0.0f}, config.dt, config.model_path) {}


    std::vector<float> evaluate(const ISolution& solution) override {
        try {
            const NetOper& net = solution.getNetOperConst();

            Model& model = model_;
            Model::State goal = {0.0f, 0.0f, 0.0f};

            Controller controller(goal, const_cast<NetOper&>(net));
            Runner runner(model, controller);
            runner.setGoal(goal);

            std::vector<Model::State> init_states = config_.generateTrainTrajectories();

            float total_score = 0.0f;
            int successes = 0;

            for (const auto& init_state : init_states) {
                total_score += scoreTrajectory(runner, model, controller, init_state, goal, successes);
            }

            return {total_score};

        } catch (const std::exception& e) {
            std::cerr << "Error in RobotFitnessEvaluator::evaluate(): " << e.what() << std::endl;
            return {1e9f};
        }
    }

    /**
     * @brief Агрегированные метрики на наборе стартов (для валидации в логе)
     */
    struct ValidMetrics {
        int total = 0;
        int successes = 0;
        int stagnated = 0;
        int invalid = 0;
        float mean_final_dist = 0.0f;
        float mean_min_dist = 0.0f;
    };

    ValidMetrics evaluateOn(NetOper& net, const std::vector<Model::State>& starts) {
        ValidMetrics m;
        m.total = static_cast<int>(starts.size());
        if (m.total == 0) return m;

        Model::State goal = {0.0f, 0.0f, 0.0f};
        Controller controller(goal, net);
        Runner runner(model_, controller);
        runner.setGoal(goal);

        float sum_final = 0.0f, sum_min = 0.0f;
        for (const auto& s : starts) {
            int succ = 0;
            // reuse scoring path but only collect counts via a light rollout
            runner.init(s);
            const float start_dist = s.distXY(goal);
            float min_dist = start_dist;
            float curr_time = 0.0f;
            float last_improve = 0.0f;
            Model::State curr = s;
            bool valid = true;
            bool stag = false;
            while (curr_time < config_.time_limit) {
                curr = runner.makeStep();
                if (!std::isfinite(curr.x) || !std::isfinite(curr.y) || !std::isfinite(curr.yaw)) {
                    valid = false;
                    break;
                }
                float d = curr.distXY(goal);
                if (d < min_dist) { min_dist = d; last_improve = curr_time; }
                curr_time += config_.dt;
                if (d < config_.epsilon_term) { succ++; break; }
                if (curr_time - last_improve > 3.0f) { stag = true; break; }
            }
            if (!valid) { m.invalid++; sum_final += start_dist; sum_min += start_dist; continue; }
            if (succ) m.successes++;
            if (stag) m.stagnated++;
            sum_final += curr.distXY(goal);
            sum_min += min_dist;
        }
        m.mean_final_dist = sum_final / m.total;
        m.mean_min_dist = sum_min / m.total;
        return m;
    }

private:
    /// Штраф за лимит-цикл: путь без приближения
    static float cyclePenalty(float path_length, float progress) {
        const float denom = std::max(progress, 0.1f);
        float pen = path_length / denom;
        return std::min(pen, 20.0f);  // cap
    }

    float scoreTrajectory(Runner& runner, Model& model, Controller& controller,
                          const Model::State& init_state, const Model::State& goal,
                          int& successes) {
        runner.init(init_state);

        const float start_dist = init_state.distXY(goal);
        float curr_time = 0.0f;
        float path_length = 0.0f;
        float min_dist = start_dist;
        float last_improve_time = 0.0f;
        float speed_near_goal = 0.0f;

        bool trajectory_valid = true;
        bool stagnated = false;
        Model::State prev_state = init_state;
        Model::State currState = init_state;

        const float stagnation_window = 3.0f;   // сек без улучшения min_dist
        const float near_goal_r = 1.0f;

        while (curr_time < config_.time_limit) {
            currState = runner.makeStep();

            if (!std::isfinite(currState.x) || !std::isfinite(currState.y) ||
                !std::isfinite(currState.yaw)) {
                trajectory_valid = false;
                break;
            }

            float dx = currState.x - prev_state.x;
            float dy = currState.y - prev_state.y;
            float step_dist = std::sqrt(dx * dx + dy * dy);
            path_length += step_dist;

            const float d = currState.distXY(goal);
            if (d < min_dist) {
                min_dist = d;
                last_improve_time = curr_time;
            }

            if (d < near_goal_r) {
                speed_near_goal += step_dist / config_.dt;
            }

            prev_state = currState;
            curr_time += config_.dt;

            if (d < config_.epsilon_term) {
                successes++;
                break;
            }

            // Early abort: долго не приближаемся
            if (curr_time - last_improve_time > stagnation_window) {
                stagnated = true;
                break;
            }
        }

        const float final_dist = currState.distXY(goal);
        const float progress = std::max(0.0f, start_dist - final_dist);

        float heading = 0.0f;
        float score = 0.0f;

        if (!trajectory_valid) {
            score = 200.0f;  // крупный штраф за NaN
            return score;
        }

        // Heading: смотрит ли на цель в конце
        {
            float dx_goal = goal.x - currState.x;
            float dy_goal = goal.y - currState.y;
            float desired_yaw = std::atan2(dy_goal, dx_goal);
            heading = std::abs(Controller::wrapAngle(currState.yaw - desired_yaw));
        }

        if (final_dist < config_.epsilon_term) {
            // Успех: малый остаток, без тяжёлых штрафов
            score = 2.0f * final_dist + 0.5f * heading;
        } else {
            score = 2.0f * final_dist
                  + 5.0f * heading
                  + 3.0f * (final_dist - min_dist)          // подлетел и ушёл / орбита
                  + 1.5f * cyclePenalty(path_length, progress)
                  + 2.0f * speed_near_goal                  // не гасит у цели
                  + (stagnated ? 20.0f : 0.0f);             // залипание
        }

        return score;
    }

    RobotProblemConfig config_;
    Model model_;
};
