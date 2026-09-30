// GAConfig.hpp
#pragma once
#include <functional>
#include <memory>
#include "nop.hpp"
#include "ifitness_evaluator.hpp"
#include "isolution.hpp"
#include <random>

struct GAConfig {
    // === Параметры GA (универсальные) ===
    int population_size = 1000;
    int num_generations = 32;
    int num_crossovers_per_gen = 24;
    float mutation_prob = 0.4f;
    float selection_alpha = 0.7f;
    int search_neighbors = 8;
    uint32_t seed = std::mt19937::default_seed;
    
    // === Кодирование хромосом ===
    int num_params = 4;      // m_p
    int int_bits = 4;
    int frac_bits = 8;
    int num_struct_variations = 10;
    
    // Network topology
    std::vector<int> nodes_for_vars = {0, 1, 2};
    std::vector<int> nodes_for_params = {3, 4, 5, 6};
    std::vector<int> nodes_for_output = {30, 31};

    std::shared_ptr<NetOper> nop_template;

    // === Интерфейсы (инъекция зависимостей) ===
    std::shared_ptr<IFitnessEvaluator> fitness_evaluator;
    std::function<std::unique_ptr<ISolution>()> solution_factory;
    
    // === Колбэки ===
    std::function<void(int gen, float avg_fitness)> on_generation_end = nullptr;
    std::function<void(const ISolution& best_solution)> on_algorithm_end = nullptr;
    /// Вызывается каждый validation_interval поколений с лучшим NetOper
    std::function<void(int gen, const NetOper& best_nop)> on_validation = nullptr;
    int validation_interval = 10;
};
