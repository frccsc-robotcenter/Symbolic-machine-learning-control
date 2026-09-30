/**
 * @file RobotProblemConfig.hpp
 * @brief Конфигурация для задачи робота
 */

#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <set>
#include <cmath>
#include <algorithm>
#include <random>
#include "base_config.hpp"
#include "model.hpp"

struct RobotProblemConfig : public BaseConfig {
    // ===== СПЕЦИФИЧНЫЕ ПАРАМЕТРЫ ДЛЯ РОБОТА =====
    
    /// Шаг симуляции (dt)
    float dt = 0.033333f;
    
    /// Максимальное время симуляции
    float time_limit = 15.0f;
    
    /// Условие остановки
    float epsilon_term = 0.1f;
    
    /// Количество траекторий для обучения
    int num_trajectories = 8;
    
    /// Минимальные значения начального состояния
    std::vector<float> qyminc = {-5.5f, -5.5f, -1.31f};
    
    /// Максимальные значения начального состояния
    std::vector<float> qymaxc = {5.5f, 5.5f, 1.31f};
    
    /// Путь к модели ONNX
    std::string model_path = "rosbot_gazebo9_2d_model.onnx";
    
    /// Количество стартовых точек для сохранения результатов
    int num_test_trajectories = 16;
    
    /// Кастомные загруженные траектории (опционально)
    std::vector<Model::State> custom_train_trajectories;

    /// Флаг использования кастомных траекторий
    bool use_custom_trajectories = false;

    /// Seed для генерации случайных стартовых состояний
    unsigned int train_seed = 42;
    
    
    // ===== ИНИЦИАЛИЗАЦИЯ =====
    
    RobotProblemConfig() {
        // Инициализируем базовую матрицу
    // base_matrix = {
    //     {1,0,0,0,0,0,1,10,0,0,12,1,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,19,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,1,0,0,0,0,0,1,0,0,0,0,0,2,9,0,0,0,0,10,0,0,0,0,1,0,0,0,0,0,0},
    //     {0,0,0,1,0,0,1,0,0,0,6,0,0,0,0,13,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,1,0,0,1,0,0,22,0,0,0,0,11,0,0,0,25,0,0,19,0,0,0,0,0,0,0,0},
    //     {0,0,1,0,0,0,6,0,0,8,0,5,0,4,13,10,0,0,0,14,15,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,2,0,1,10,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,2,1,0,0,8,0,0,0,12,0,0,1,19,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,0,2,0,0,0,1,1,8,26,0,0,6,8,0,0,0,14,12,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,7,1,26,5,4,23,0,0,0,0,15,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,7,17,10,0,0,0,0,0,16,0,16,0,16,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,14,0,25,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,7,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,7,10,0,1,0,0,14,0,13,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,8,0,0,27,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,1,0,0,0,0,0,0,0,0,0,0,22,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,15,4,0,0,0,0,0,0,0,0,0,0},
    //     {0,7,0,0,0,0,0,0,0,0,17,0,0,0,1,0,0,0,7,0,0,17,13,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,21,0,16,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,1,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,1,16,16,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,4,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,7,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,7,18},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1}
    // };

    base_matrix = {
        {1,0,0,0,0,0,1,10,0,0,12,1,0,0,0,0,0,0,0,0,0,0,0,10},
        {0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,19,0,0,0,0},
        {0,0,1,0,0,0,0,0,1,0,0,0,0,0,2,9,0,0,0,0,10,0,0,0},
        {0,0,0,1,0,0,1,0,0,0,6,0,0,0,0,13,0,0,0,0,0,0,0,0},
        {0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,1,0,0,22,0,0,0,0,11,0,0,0,25,0,0,19},
        {0,0,1,0,0,0,6,0,0,8,0,5,0,4,13,10,0,0,0,14,15,0,0,0},
        {0,0,0,0,0,0,0,2,0,1,10,9,0,0,0,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,2,1,0,0,8,0,0,0,12,0,0,1,19,0,0,0},
        {0,1,0,0,0,2,0,0,0,1,1,8,26,0,0,6,8,0,0,0,14,12,0,0},
        {0,0,0,0,0,0,0,0,0,0,7,1,26,5,4,23,0,0,0,0,15,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,7,17,10,0,0,0,0,0,16,0,16,0,16},
        {0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,14,0,25,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,7,9,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,7,10,0,1,0,0,14,0,13},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,8,0,0,27,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,1,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,15,4,0,0},
        {0,7,0,0,0,0,0,0,0,0,17,0,0,0,1,0,0,0,7,0,0,17,13,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,21,0,16,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,1,0,0},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,1,16,16},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,4},
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6}
    };

    base_params = {50150.5, 23227.4, 5022.71, 37037.5, 376.38, 2979.02, 14908.7, 27469.4};
    
    nodes_for_vars = {0, 1, 2};
    nodes_for_params = {3, 4, 5, 6, 7, 8, 9, 10};
    nodes_for_output = {22, 23};
}
    
    
    /**
     * @brief Загрузить траектории из CSV файла
     * 
     * Формат CSV: Trajectory,Time,X,Y,Theta
     * Загружает начальные состояния всех уникальных траекторий
     */
    void loadTrajectories(const std::string& csv_file) {
        custom_train_trajectories.clear();
        std::set<int> loaded_trajectories;
        
        std::ifstream file(csv_file);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << csv_file << std::endl;
            return;
        }
        
        std::string line;
        // Пропускаем заголовок
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            
            std::stringstream ss(line);
            int traj_id;
            float time, x, y, theta;
            char comma;
            
            // Парсим CSV: Trajectory,Time,X,Y,Theta
            ss >> traj_id >> comma >> time >> comma >> x >> comma >> y >> comma >> theta;
            
            // Добавляем только начальное состояние каждой траектории (время ≈ 0)
            if (time < 0.01f && loaded_trajectories.find(traj_id) == loaded_trajectories.end()) {
                custom_train_trajectories.push_back(Model::State{x, y, theta});
                loaded_trajectories.insert(traj_id);
            }
        }
        
        file.close();
        
        if (!custom_train_trajectories.empty()) {
            use_custom_trajectories = true;
            num_trajectories = custom_train_trajectories.size();
            std::cout << "Loaded " << num_trajectories << " trajectories from " << csv_file << std::endl;
        }
    }
    
    
    /**
     * @brief Генерировать разнообразные стартовые состояния для обучения
     *
     * Слои: 8 углов куба, осевые старты (θ=0), кольцо по окружности,
     * случайные равномерные точки. Без дублирования углов.
     */
    std::vector<Model::State> generateTrainTrajectories() const {
        if (use_custom_trajectories && !custom_train_trajectories.empty()) {
            return custom_train_trajectories;
        }

        std::vector<Model::State> states;
        states.reserve(static_cast<size_t>(std::max(1, num_trajectories)));

        const float x_min = qyminc[0], x_max = qymaxc[0];
        const float y_min = qyminc[1], y_max = qymaxc[1];
        const float th_min = qyminc[2], th_max = qymaxc[2];
        const float x_mid = 0.5f * (x_min + x_max);
        const float y_mid = 0.5f * (y_min + y_max);
        const float th_mid = 0.5f * (th_min + th_max);

        // 1) 8 углов куба (обязательный слой)
        for (int i = 0; i < 8; ++i) {
            states.push_back(Model::State{
                (i & 4) ? x_max : x_min,
                (i & 2) ? y_max : y_min,
                (i & 1) ? th_max : th_min
            });
        }

        // 2) Осевые старты с θ = th_mid (обычно 0) — критично для «залипания»
        const float axis_dist = 0.9f * std::max(std::fabs(x_max), std::fabs(y_max));
        const float axis_mid = 0.45f * axis_dist;
        for (float d : {axis_dist, axis_mid}) {
            states.push_back(Model::State{ d,  y_mid, th_mid});
            states.push_back(Model::State{-d,  y_mid, th_mid});
            states.push_back(Model::State{x_mid,  d,  th_mid});
            states.push_back(Model::State{x_mid, -d,  th_mid});
        }

        // 3) Кольцо по окружности, случайный курс
        std::mt19937 rng(train_seed);
        std::uniform_real_distribution<float> u01(0.0f, 1.0f);
        for (float r : {1.0f, 3.0f, 5.0f}) {
            for (int k = 0; k < 8; ++k) {
                const float ang = (static_cast<float>(k) / 8.0f) * 2.0f * 3.14159265f;
                states.push_back(Model::State{
                    r * std::cos(ang),
                    r * std::sin(ang),
                    th_min + u01(rng) * (th_max - th_min)
                });
            }
        }

        // 4) Случайные равномерные точки до num_trajectories
        std::uniform_real_distribution<float> ux(x_min, x_max);
        std::uniform_real_distribution<float> uy(y_min, y_max);
        std::uniform_real_distribution<float> uth(th_min, th_max);
        while (static_cast<int>(states.size()) < num_trajectories) {
            states.push_back(Model::State{ux(rng), uy(rng), uth(rng)});
        }

        // Если num_trajectories меньше структурного минимума — сохраняем
        // углы и оси (критично против залипания), усекая кольцо/случайные.
        const size_t keep_min = 16; // 8 углов + 8 осевых
        if (static_cast<int>(states.size()) > num_trajectories) {
            if (static_cast<size_t>(num_trajectories) >= keep_min) {
                states.resize(static_cast<size_t>(num_trajectories));
            } else {
                states.resize(keep_min);
            }
        }
        return states;
    }
    
    
    /**
     * @brief Генерировать стартовые траектории для тестирования/сохранения
     * Генерирует num_test_trajectories точек по краям области
     * @return Вектор начальных состояний распределённых по краям
     */
    std::vector<Model::State> generateTestTrajectories() const {
        std::vector<Model::State> states;
        states.reserve(num_test_trajectories);
                
        int points_per_edge = num_test_trajectories / 4;
        int remaining = num_test_trajectories % 4;
        
        // Параметры области
        float x_min = qyminc[0], x_max = qymaxc[0];
        float y_min = qyminc[1], y_max = qymaxc[1];
        float theta_min = qyminc[2], theta_max = qymaxc[2];
        
        float theta_mid = (theta_min + theta_max) / 2.0f;
        
        int count = 0;
        
        // НИЖНИЙ край (y_min)
        for (int i = 0; i < points_per_edge + (remaining > 0 ? 1 : 0) && count < num_test_trajectories; ++i, ++count) {
            float t = (points_per_edge > 0) ? static_cast<float>(i) / points_per_edge : 0;
            float x = x_min + t * (x_max - x_min);
            states.push_back(Model::State{x, y_min, theta_mid});
        }
        remaining = std::max(0, remaining - 1);
        
        // ВЕРХНИЙ край (y_max)
        for (int i = 0; i < points_per_edge + (remaining > 0 ? 1 : 0) && count < num_test_trajectories; ++i, ++count) {
            float t = (points_per_edge > 0) ? static_cast<float>(i) / points_per_edge : 0;
            float x = x_max - t * (x_max - x_min);  // идём в обратном направлении
            states.push_back(Model::State{x, y_max, theta_mid});
        }
        remaining = std::max(0, remaining - 1);
        
        // ЛЕВЫЙ край (x_min)
        for (int i = 0; i < points_per_edge + (remaining > 0 ? 1 : 0) && count < num_test_trajectories; ++i, ++count) {
            float t = (points_per_edge > 0) ? static_cast<float>(i) / points_per_edge : 0;
            float y = y_min + t * (y_max - y_min);
            states.push_back(Model::State{x_min, y, theta_mid});
        }
        remaining = std::max(0, remaining - 1);
        
        // ПРАВЫЙ край (x_max)
        for (int i = 0; i < points_per_edge + (remaining > 0 ? 1 : 0) && count < num_test_trajectories; ++i, ++count) {
            float t = (points_per_edge > 0) ? static_cast<float>(i) / points_per_edge : 0;
            float y = y_max - t * (y_max - y_min);  // идём в обратном направлении
            states.push_back(Model::State{x_max, y, theta_mid});
        }
        
        // Если ещё есть точки, добавляем их с разными углами по краям
        while (states.size() < static_cast<size_t>(num_test_trajectories)) {
            int idx = states.size() % 4;
            float theta_var = theta_min + (idx * (theta_max - theta_min) / 4.0f);
            
            if (idx == 0) {
                states.push_back(Model::State{x_min, (y_min + y_max) / 2.0f, theta_var});
            } else if (idx == 1) {
                states.push_back(Model::State{x_max, (y_min + y_max) / 2.0f, theta_var});
            } else if (idx == 2) {
                states.push_back(Model::State{(x_min + x_max) / 2.0f, y_min, theta_var});
            } else {
                states.push_back(Model::State{(x_min + x_max) / 2.0f, y_max, theta_var});
            }
        }
        
        return states;
    }


};
