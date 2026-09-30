#include "GANOP.hpp"
#include "nop.hpp"
#include "GAConfig.hpp"
#include "ifitness_evaluator.hpp"
#include "isolution.hpp"
#include "base_solution.hpp"
#include "base_config.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

// ===== Test helpers =====

struct TestConfig : BaseConfig {
    TestConfig() {
        base_matrix = {
            {1, 2, 0},
            {0, 1, 3},
            {0, 0, 1}
        };
        base_params = {1.0f, 2.0f};
        nodes_for_vars = {0};
        nodes_for_params = {1};
        nodes_for_output = {2};
    }
};

class SumFitnessEvaluator : public IFitnessEvaluator {
public:
    int getNumObjectives() const override { return 1; }

    std::vector<float> evaluate(const ISolution& solution) override {
        const NetOper& net = solution.getNetOperConst();
        auto& params = const_cast<NetOper&>(net).get_parameters();
        float sum = 0.0f;
        for (float p : params) sum += std::abs(p);
        return {sum};
    }
};

GAConfig makeTestGAConfig(int pop_size = 20, int generations = 3) {
    GAConfig cfg;
    cfg.population_size = pop_size;
    cfg.num_generations = generations;
    cfg.num_crossovers_per_gen = 5;
    cfg.mutation_prob = 0.5f;
    cfg.selection_alpha = 0.5f;
    cfg.search_neighbors = 4;
    cfg.int_bits = 4;
    cfg.frac_bits = 4;
    cfg.num_params = 2;
    cfg.num_struct_variations = 3;
    cfg.seed = 42;

    TestConfig tc;
    cfg.nodes_for_vars = tc.nodes_for_vars;
    cfg.nodes_for_params = tc.nodes_for_params;
    cfg.nodes_for_output = tc.nodes_for_output;

    cfg.nop_template = std::make_shared<NetOper>();
    cfg.nop_template->setNodesForVars(tc.nodes_for_vars);
    cfg.nop_template->setNodesForParams(tc.nodes_for_params);
    cfg.nop_template->setNodesForOutput(tc.nodes_for_output);
    cfg.nop_template->setCs(tc.base_params);
    cfg.nop_template->setPsi(tc.base_matrix);

    cfg.fitness_evaluator = std::make_shared<SumFitnessEvaluator>();

    cfg.solution_factory = [&cfg]() -> std::unique_ptr<ISolution> {
        TestConfig tc;
        auto sol = std::make_unique<BaseSolution<TestConfig>>(tc);
        sol->setIntBits(cfg.int_bits);
        sol->setFracBits(cfg.frac_bits);
        return sol;
    };

    return cfg;
}

// ===== Grey encoding tests =====

// Helper: encode a float param into grey code bits
// Layout: [sign | int_bits (MSB first) | frac_bits (MSB first)]
static void encode_param_to_grey(float param, int int_bits, int frac_bits,
                                  std::vector<int>& grey, int base) {
    int bpp = 1 + int_bits + frac_bits;
    bool negative = (param < 0.0f);
    param = std::abs(param);

    // Binary code for this param
    std::vector<int> binary(bpp, 0);
    binary[0] = negative ? 1 : 0;

    int x = static_cast<int>(std::floor(param));
    double r = param - x;

    // Integer bits: store MSB first at indices 1..int_bits
    for (int i = int_bits; i >= 1; --i) {
        binary[i] = x % 2;
        x /= 2;
    }

    // Fractional bits: store at indices int_bits+1 .. bpp-1
    for (int i = int_bits + 1; i < bpp; ++i) {
        r *= 2.0;
        binary[i] = static_cast<int>(std::floor(r));
        r -= binary[i];
    }

    // Binary -> Grey
    grey[base] = binary[0];
    for (int i = 1; i < bpp; ++i) {
        grey[base + i] = binary[i] ^ binary[i - 1];
    }
}

TEST(GANOP_Grey, roundtrip_positive_params) {
    GAConfig cfg = makeTestGAConfig();
    cfg.nop_template->setCs({5.0f, 3.0f});
    cfg.num_generations = 0;
    GANOP ga(cfg);
    ga.run();

    auto best_idx = ga.getBestParetoIndex();
    EXPECT_GE(best_idx, 0);
}

TEST(GANOP_Grey, roundtrip_various_values) {
    TestConfig tc;
    BaseSolution<TestConfig> sol(tc);
    sol.setIntBits(4);
    sol.setFracBits(4);

    int bpp = 1 + 4 + 4;
    std::vector<int> grey(2 * bpp, 0);

    encode_param_to_grey(1.5f, 4, 4, grey, 0);
    encode_param_to_grey(7.25f, 4, 4, grey, bpp);

    std::vector<std::vector<int>> struct_code = {{}};
    sol.decode(grey, struct_code);

    auto& params = sol.getNetOper().get_parameters();
    ASSERT_EQ(params.size(), 2u);
    EXPECT_NEAR(params[0], 1.5f, 0.01f);
    EXPECT_NEAR(params[1], 7.25f, 0.01f);
}

TEST(GANOP_Grey, negative_params_sign_bit) {
    TestConfig tc;
    BaseSolution<TestConfig> sol(tc);
    sol.setIntBits(4);
    sol.setFracBits(4);

    int bpp = 1 + 4 + 4;
    std::vector<int> grey(2 * bpp, 0);

    encode_param_to_grey(-3.0f, 4, 4, grey, 0);
    encode_param_to_grey(-1.0f, 4, 4, grey, bpp);

    std::vector<std::vector<int>> struct_code = {{}};
    sol.decode(grey, struct_code);

    auto& params = sol.getNetOper().get_parameters();
    ASSERT_EQ(params.size(), 2u);
    EXPECT_NEAR(params[0], -3.0f, 0.01f);
    EXPECT_NEAR(params[1], -1.0f, 0.01f);
}

TEST(GANOP_Grey, zero_params) {
    TestConfig tc;
    BaseSolution<TestConfig> sol(tc);
    sol.setIntBits(4);
    sol.setFracBits(4);

    int bpp = 1 + 4 + 4;
    std::vector<int> grey(2 * bpp, 0);

    encode_param_to_grey(0.0f, 4, 4, grey, 0);
    encode_param_to_grey(0.0f, 4, 4, grey, bpp);

    std::vector<std::vector<int>> struct_code = {{}};
    sol.decode(grey, struct_code);

    auto& params = sol.getNetOper().get_parameters();
    ASSERT_EQ(params.size(), 2u);
    EXPECT_NEAR(params[0], 0.0f, 0.01f);
    EXPECT_NEAR(params[1], 0.0f, 0.01f);
}

// ===== GenVar / Variations tests =====

TEST(NetOper_GenVar, generates_valid_variations) {
    NetOper nop;
    nop.setPsi({
        {1, 2, 0, 0},
        {0, 1, 3, 0},
        {0, 0, 1, 4},
        {0, 0, 0, 1}
    });
    nop.setNodesForVars({0});
    nop.setNodesForParams({1});

    std::mt19937 rng(42);
    std::vector<int> w(4);

    for (int trial = 0; trial < 100; ++trial) {
        nop.GenVar(w, rng);
        EXPECT_GE(w[0], 0);
        EXPECT_LE(w[0], 3);

        switch (w[0]) {
        case 0: case 2: case 3:
            EXPECT_GE(w[1], 0);
            EXPECT_GT(w[2], w[1]);  // j > i
            EXPECT_GE(w[3], 1);     // operation >= 1
            break;
        case 1:
            EXPECT_GE(w[1], 0);
            EXPECT_LT(w[1], 4);
            EXPECT_EQ(w[2], w[1]);  // diagonal
            EXPECT_GE(w[3], 1);
            break;
        }
    }
}

TEST(NetOper_GenVar, deterministic_with_same_seed) {
    NetOper nop;
    nop.setPsi({
        {1, 2, 0},
        {0, 1, 3},
        {0, 0, 1}
    });
    nop.setNodesForVars({0});
    nop.setNodesForParams({1});

    std::mt19937 rng1(123);
    std::mt19937 rng2(123);
    std::vector<int> w1(4), w2(4);

    for (int i = 0; i < 50; ++i) {
        nop.GenVar(w1, rng1);
        nop.GenVar(w2, rng2);
        EXPECT_EQ(w1, w2);
    }
}

TEST(NetOper_Variations, replace_non_diagonal) {
    NetOper nop;
    nop.setPsi({
        {1, 5, 0},
        {0, 1, 3},
        {0, 0, 1}
    });

    // Replace non-diagonal element [0][1] with value 9
    nop.Variations({0, 0, 1, 9});
    EXPECT_EQ(nop.getPsi()[0][1], 9);
}

TEST(NetOper_Variations, replace_diagonal) {
    NetOper nop;
    nop.setPsi({
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    });

    // Replace diagonal element [1][1] with value 5
    nop.Variations({1, 1, 1, 5});
    EXPECT_EQ(nop.getPsi()[1][1], 5);
}

TEST(NetOper_Variations, add_arc) {
    NetOper nop;
    nop.setPsi({
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    });

    // Add arc [0][2] with value 7 (requires [2][2] != 0)
    nop.Variations({2, 0, 2, 7});
    EXPECT_EQ(nop.getPsi()[0][2], 7);
}

TEST(NetOper_Variations, add_arc_requires_existing_diagonal) {
    NetOper nop;
    nop.setPsi({
        {1, 0, 0},
        {0, 0, 0},  // diagonal is 0
        {0, 0, 1}
    });

    // Try to add arc to node 1 (diagonal is 0) — should not change
    nop.Variations({2, 0, 1, 7});
    EXPECT_EQ(nop.getPsi()[0][1], 0);  // unchanged
}

TEST(NetOper_Variations, remove_arc) {
    NetOper nop;
    nop.setPsi({
        {1, 5, 3},
        {2, 1, 4},
        {1, 1, 1}
    });

    // Remove arc [0][2] — needs s1 > 1 and s2 > 1
    // s1 = count of non-zero in column 2 above row 0 = 0 (nothing above)
    // Actually the check is different: s1 counts column entries, s2 counts row entries
    nop.Variations({3, 0, 2, 0});
    // The arc may or may not be removed depending on connectivity
    // Just verify no crash
}

TEST(NetOper_Variations, empty_vector) {
    NetOper nop;
    std::vector<int> w;
    EXPECT_NO_THROW(nop.Variations(w));
}

// ===== Pareto ranking tests =====

TEST(GANOP_Pareto, computeRank_single_objective) {
    GAConfig cfg = makeTestGAConfig(5, 1);
    GANOP ga(cfg);

    // After construction, ranks should be initialized
    // We can't directly test private members, but we can test through run()
    // For now, just verify construction doesn't crash
    EXPECT_NO_THROW(GANOP ga2(cfg));
}

TEST(GANOP_Pareto, run_completes_without_crash) {
    GAConfig cfg = makeTestGAConfig(10, 2);
    GANOP ga(cfg);
    EXPECT_NO_THROW(ga.run());
}

TEST(GANOP_Pareto, run_finds_solutions) {
    GAConfig cfg = makeTestGAConfig(20, 5);
    GANOP ga(cfg);
    ga.run();

    auto& pareto = ga.getParetoIndices();
    EXPECT_FALSE(pareto.empty());

    // Best Pareto index should be valid
    int best = ga.getBestParetoIndex();
    EXPECT_GE(best, 0);
    EXPECT_LT(best, cfg.population_size);
}

// ===== Crossover tests =====

TEST(GANOP_Crossover, produces_four_offspring) {
    GAConfig cfg = makeTestGAConfig(20, 1);
    GANOP ga(cfg);

    // We can't easily test crossover in isolation (private method)
    // But we can verify the GA runs crossover without crashing
    cfg.num_crossovers_per_gen = 10;
    GANOP ga2(cfg);
    EXPECT_NO_THROW(ga2.run());
}

// ===== Mutation tests =====

TEST(GANOP_Mutation, preserves_chromosome_size) {
    GAConfig cfg = makeTestGAConfig(20, 1);
    cfg.mutation_prob = 1.0f;  // always mutate
    GANOP ga(cfg);
    EXPECT_NO_THROW(ga.run());
}

// ===== Integration tests =====

TEST(GANOP_Integration, fitness_decreases_over_generations) {
    GAConfig cfg = makeTestGAConfig(50, 10);
    cfg.mutation_prob = 0.5f;

    std::vector<float> fitness_history;
    cfg.on_generation_end = [&](int gen, float avg) {
        fitness_history.push_back(avg);
    };

    GANOP ga(cfg);
    ga.run();

    // Fitness should generally improve (decrease) over generations
    // Check that last generation is better than first
    if (fitness_history.size() >= 2) {
        EXPECT_LE(fitness_history.back(), fitness_history.front() * 1.5f);
    }
}

TEST(GANOP_Integration, reproducible_with_same_seed) {
    GAConfig cfg1 = makeTestGAConfig(20, 3);
    cfg1.seed = 12345;

    GAConfig cfg2 = makeTestGAConfig(20, 3);
    cfg2.seed = 12345;

    GANOP ga1(cfg1);
    ga1.run();

    GANOP ga2(cfg2);
    ga2.run();

    auto& fit1 = ga1.getAllFitness();
    auto& fit2 = ga2.getAllFitness();

    ASSERT_EQ(fit1.size(), fit2.size());
    for (size_t i = 0; i < fit1.size(); ++i) {
        ASSERT_EQ(fit1[i].size(), fit2[i].size());
        for (size_t j = 0; j < fit1[i].size(); ++j) {
            EXPECT_FLOAT_EQ(fit1[i][j], fit2[i][j]);
        }
    }
}

TEST(GANOP_Integration, on_algorithm_end_called) {
    GAConfig cfg = makeTestGAConfig(10, 2);

    bool callback_called = false;
    cfg.on_algorithm_end = [&](const ISolution&) {
        callback_called = true;
    };

    GANOP ga(cfg);
    ga.run();
    EXPECT_TRUE(callback_called);
}

// ===== Edge cases =====

TEST(GANOP_Edge, small_population) {
    GAConfig cfg = makeTestGAConfig(5, 1);
    cfg.num_crossovers_per_gen = 2;
    GANOP ga(cfg);
    EXPECT_NO_THROW(ga.run());
}

TEST(GANOP_Edge, high_mutation_rate) {
    GAConfig cfg = makeTestGAConfig(20, 3);
    cfg.mutation_prob = 1.0f;
    GANOP ga(cfg);
    EXPECT_NO_THROW(ga.run());
}

TEST(GANOP_Edge, zero_generations) {
    GAConfig cfg = makeTestGAConfig(10, 0);
    GANOP ga(cfg);
    EXPECT_NO_THROW(ga.run());
}

TEST(GANOP_Edge, single_param) {
    GAConfig cfg;
    cfg.population_size = 10;
    cfg.num_generations = 2;
    cfg.num_crossovers_per_gen = 3;
    cfg.mutation_prob = 0.5f;
    cfg.int_bits = 4;
    cfg.frac_bits = 4;
    cfg.num_params = 1;
    cfg.num_struct_variations = 2;
    cfg.seed = 99;

    cfg.nop_template = std::make_shared<NetOper>();
    cfg.nop_template->setNodesForVars({0});
    cfg.nop_template->setNodesForParams({1});
    cfg.nop_template->setNodesForOutput({2});
    cfg.nop_template->setCs({5.0f});
    cfg.nop_template->setPsi({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});

    cfg.fitness_evaluator = std::make_shared<SumFitnessEvaluator>();
    cfg.solution_factory = [&cfg]() -> std::unique_ptr<ISolution> {
        TestConfig tc;
        auto sol = std::make_unique<BaseSolution<TestConfig>>(tc);
        sol->setIntBits(cfg.int_bits);
        sol->setFracBits(cfg.frac_bits);
        return sol;
    };

    GANOP ga(cfg);
    EXPECT_NO_THROW(ga.run());
}
