#include "GANOP.hpp"
#include "RobotFitnessEvaluator.hpp"
#include "base_solution.hpp"
#include "RobotProblemConfig.hpp"
#include "p_controller_seed.hpp"
#include "controller.hpp"
#include "runner.hpp"
#include "model.hpp"
#include <iostream>
#include <fstream>
#include <memory>


// Глобальная конфигурация для доступа в колбэке
RobotProblemConfig g_robot_config;
GAConfig g_ga_config;
std::shared_ptr<RobotFitnessEvaluator> g_fitness;


void log_generation(int gen, float avg_fitness) {
    static std::ofstream log("evolution_log.txt", std::ios::app);
    log << "Generation " << gen << ": avg_fitness = " << avg_fitness << std::endl;
    std::cout << "Gen " << gen << " done, avg: " << avg_fitness << std::endl;
}


void log_validation(int gen, const NetOper& best_nop) {
    static std::ofstream log("evolution_log.txt", std::ios::app);
    // NetOper::calcResult не const — нужен const_cast для копии
    NetOper copy = best_nop;
    auto starts = g_robot_config.generateTestTrajectories();
    auto m = g_fitness->evaluateOn(copy, starts);

    log << "VALIDATION gen " << gen
        << ": success=" << m.successes << "/" << m.total
        << " stagnated=" << m.stagnated
        << " invalid=" << m.invalid
        << " mean_final_dist=" << m.mean_final_dist
        << " mean_min_dist=" << m.mean_min_dist
        << std::endl;

    std::cout << "  [val " << gen << "] success " << m.successes << "/" << m.total
              << "  mean_final=" << m.mean_final_dist
              << "  mean_min=" << m.mean_min_dist
              << "  stuck=" << m.stagnated
              << std::endl;
}


bool file_exists(const std::string& filename) {
    std::ifstream infile(filename);
    return infile.good();
}


/**
 * @brief Сохранить лучшее решение + финальные метрики на тесте
 */
void save_best_solution(const ISolution& solution) {
    const NetOper& net = solution.getNetOperConst();

    std::cout << "\n=== BEST SOLUTION FOUND ===" << std::endl;
    std::cout << "Best NOP matrix" << std::endl;
    net.printMatrix();

    if (!net.saveMatrixToFile("best_matrix.txt")) {
        std::cerr << "WARNING: Failed to save best_matrix.txt" << std::endl;
    }

    if (!net.saveParametersToFile("best_params.txt")) {
        std::cerr << "WARNING: Failed to save best_params.txt" << std::endl;
    }

    // Симулируем траектории и сохраняем
    std::ofstream outFile("trajectories.csv");
    if (!outFile.is_open()) {
        std::cerr << "Failed to open trajectories.csv for writing!" << std::endl;
        return;
    }

    outFile << "Trajectory,Time,X,Y,Theta\n";

    Model::State currState = {0.0f, 0.0f, 0.0f};
    Model model(currState, g_robot_config.dt, g_robot_config.model_path);
    Model::State goal = {0.0f, 0.0f, 0.0f};

    // const_cast needed: Controller stores NetOper& and calcResult modifies internal z buffer
    Controller controller(goal, const_cast<NetOper&>(net));
    Runner runner(model, controller);
    runner.setGoal(goal);

    std::vector<Model::State> test_states = g_robot_config.generateTestTrajectories();

    std::cout << "Simulating " << test_states.size() << " test trajectories..." << std::endl;

    int successes = 0;
    for (size_t i = 0; i < test_states.size(); ++i) {
        runner.init(test_states[i]);
        float currTime = 0.0f;

        while (currTime < g_robot_config.time_limit) {
            currState = runner.makeStep();
            outFile << i << "," << currTime << ","
                   << currState.x << "," << currState.y << ","
                   << currState.yaw << "\n";

            currTime += g_robot_config.dt;

            if (currState.distXY(goal) < g_robot_config.epsilon_term) {
                successes++;
                break;
            }
        }
    }

    outFile.close();
    std::cout << "Trajectories logged to trajectories.csv (" << test_states.size() << " trajectories)" << std::endl;
    std::cout << "TEST SUCCESS RATE: " << successes << " / " << test_states.size()
              << "  (" << (100.0f * successes / std::max(1, (int)test_states.size())) << "%)"
              << std::endl;

    // Вывод параметров сети
    std::cout << "Parameters: ";
    for (float p : const_cast<NetOper&>(net).get_parameters()) {
        std::cout << p << " ";
    }
    std::cout << std::endl;
}


int main() {
    // Чистим только логи симуляции; best_matrix/best_params сохраняем для warm-start
    for (auto f : {"evolution_log.txt", "trajectories.csv"})
        std::remove(f);

    // === 1. Конфигурация проблемы ===
    RobotProblemConfig robot_config;
    robot_config.dt = 0.033333f;
    robot_config.time_limit = 15.0f;
    robot_config.epsilon_term = 0.15f;

    robot_config.num_trajectories = 48;
    robot_config.num_test_trajectories = 64;
    robot_config.train_seed = 42;

    robot_config.qyminc = {-5.5f, -5.5f, -1.31f};
    robot_config.qymaxc = {5.5f, 5.5f, 1.31f};
    robot_config.model_path = "rosbot_gazebo9_2d_model.onnx";

    // === 2. Конфигурация GA ===
    GAConfig ga_config;

    ga_config.nodes_for_vars = robot_config.nodes_for_vars;
    ga_config.nodes_for_params = robot_config.nodes_for_params;
    ga_config.nodes_for_output = robot_config.nodes_for_output;

    ga_config.population_size = 500;
    ga_config.num_generations = 200;
    ga_config.num_crossovers_per_gen = 30;
    ga_config.mutation_prob = 1.0f;
    ga_config.selection_alpha = 0.9f;
    ga_config.search_neighbors = 64;
    ga_config.int_bits = 8;
    ga_config.frac_bits = 16;
    ga_config.num_params = 8;
    ga_config.num_struct_variations = 10;
    ga_config.seed = 69;
    ga_config.validation_interval = 10;

    // === Посев: предыдущий контроллер или П-регулятор ===
    std::cout << "\n=== LOADING NETWORK STATE ===" << std::endl;

    bool loaded_from_file = false;
    if (file_exists("best_matrix.txt") && file_exists("best_params.txt")) {
        std::cout << "Found saved network state (previous controller)." << std::endl;
        NetOper tmp;
        if (tmp.loadMatrixFromFile("best_matrix.txt") &&
            tmp.loadParametersFromFile("best_params.txt")) {
            // Warm-start: структура и параметры идут в base_* (иначе decode их теряет)
            robot_config.base_matrix = tmp.getPsi();
            robot_config.base_params = tmp.getCs();
            std::cout << "✓ Previous controller loaded into base_matrix/base_params" << std::endl;
            loaded_from_file = true;
        } else {
            std::cerr << "WARNING: failed to load best_*, falling back to P-controller" << std::endl;
        }
    }

    if (!loaded_from_file) {
        robot_config.base_matrix = p_seed::makePControllerMatrix();
        robot_config.base_params = p_seed::makePControllerParams(p_seed::kDefaultKv, p_seed::kDefaultKw);
        std::cout << "✓ Seeded with P-controller (kv=" << p_seed::kDefaultKv
                  << ", kw=" << p_seed::kDefaultKw << ")" << std::endl;
    }

    // Убедимся, что узлы заданы
    robot_config.nodes_for_vars = {0, 1, 2};
    robot_config.nodes_for_params = {3, 4, 5, 6, 7, 8, 9, 10};
    robot_config.nodes_for_output = {22, 23};

    ga_config.nop_template = std::make_shared<NetOper>();
    ga_config.nop_template->setNodesForVars(robot_config.nodes_for_vars);
    ga_config.nop_template->setNodesForParams(robot_config.nodes_for_params);
    ga_config.nop_template->setNodesForOutput(robot_config.nodes_for_output);
    ga_config.nop_template->setPsi(robot_config.base_matrix);
    ga_config.nop_template->setCs(robot_config.base_params);
    std::cout << "NetOper template initialized" << std::endl;

    // Сохраняем конфиги глобально для доступа в колбэках
    g_robot_config = robot_config;
    g_ga_config = ga_config;

    // === 3. Инъекция зависимостей ===
    g_fitness = std::make_shared<RobotFitnessEvaluator>(robot_config, 1);
    ga_config.fitness_evaluator = g_fitness;

    // Factory: берём robot_config по значению — обновляем ДО создания фабрики
    ga_config.solution_factory = [robot_config, &ga_config]() -> std::unique_ptr<ISolution> {
        auto solution = std::make_unique<BaseSolution<RobotProblemConfig>>(robot_config);
        solution->setIntBits(ga_config.int_bits);
        solution->setFracBits(ga_config.frac_bits);
        return solution;
    };

    ga_config.on_generation_end = log_generation;
    ga_config.on_algorithm_end = save_best_solution;
    ga_config.on_validation = log_validation;

    // === Бейзлайн: П-регулятор до старта GA ===
    {
        NetOper pnet;
        pnet.setNodesForVars(robot_config.nodes_for_vars);
        pnet.setNodesForParams(robot_config.nodes_for_params);
        pnet.setNodesForOutput(robot_config.nodes_for_output);
        pnet.setPsi(p_seed::makePControllerMatrix());
        pnet.setCs(p_seed::makePControllerParams());

        auto test_starts = robot_config.generateTestTrajectories();
        auto m = g_fitness->evaluateOn(pnet, test_starts);
        std::cout << "\n=== P-CONTROLLER BASELINE ===" << std::endl;
        std::cout << "success=" << m.successes << "/" << m.total
                  << "  mean_final_dist=" << m.mean_final_dist
                  << "  mean_min_dist=" << m.mean_min_dist
                  << "  stagnated=" << m.stagnated
                  << "  invalid=" << m.invalid << std::endl;

        std::ofstream log("evolution_log.txt", std::ios::app);
        log << "P_BASELINE: success=" << m.successes << "/" << m.total
            << " mean_final_dist=" << m.mean_final_dist
            << " mean_min_dist=" << m.mean_min_dist
            << " stagnated=" << m.stagnated << std::endl;
    }

    // === 4. Запуск GA ===
    std::cout << "\n=== STARTING GENETIC ALGORITHM ===" << std::endl;
    std::cout << "Population: " << ga_config.population_size << std::endl;
    std::cout << "Generations: " << ga_config.num_generations << std::endl;
    std::cout << "Training trajectories: " << robot_config.num_trajectories << std::endl;
    std::cout << "Test trajectories: " << robot_config.num_test_trajectories << std::endl;
    std::cout << "Seed source: " << (loaded_from_file ? "previous controller" : "P-controller")
              << std::endl;

    try {
        GANOP ga(ga_config);
        ga.run();

        std::cout << "\n=== GA COMPLETED SUCCESSFULLY ===" << std::endl;
        std::cout << "Results saved to:" << std::endl;
        std::cout << "  - best_matrix.txt" << std::endl;
        std::cout << "  - best_params.txt" << std::endl;
        std::cout << "  - trajectories.csv" << std::endl;
        std::cout << "  - evolution_log.txt" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
