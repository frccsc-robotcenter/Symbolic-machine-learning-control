#include "nop.hpp"
#include "nop_test_utils.h"
#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>
#include <random>

// Test file I/O operations for matrices
TEST(NOP_FileIO, save_and_load_matrix) {
    auto netOper = NetOper();
    
    // Setup test matrix
    std::vector<std::vector<int>> testMatrix = {
        {1, 0, 2, 0},
        {0, 1, 0, 3},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    
    netOper.setPsi(testMatrix);
    
    // Save to file
    std::string testFile = "/tmp/test_matrix.txt";
    EXPECT_TRUE(netOper.saveMatrixToFile(testFile));
    
    // Create new NetOper and load
    auto netOper2 = NetOper();
    EXPECT_TRUE(netOper2.loadMatrixFromFile(testFile));
    
    // Verify loaded matrix matches
    auto loadedMatrix = netOper2.getPsi();
    EXPECT_EQ(loadedMatrix.size(), testMatrix.size());
    for (size_t i = 0; i < testMatrix.size(); ++i) {
        EXPECT_EQ(loadedMatrix[i].size(), testMatrix[i].size());
        for (size_t j = 0; j < testMatrix[i].size(); ++j) {
            EXPECT_EQ(loadedMatrix[i][j], testMatrix[i][j]);
        }
    }
    
    // Cleanup
    std::remove(testFile.c_str());
}

TEST(NOP_FileIO, save_and_load_parameters) {
    auto netOper = NetOper();
    
    // Setup test parameters
    std::vector<float> testParams = {1.5f, 2.5f, 3.5f, 4.5f, 5.5f};
    netOper.setCs(testParams);
    
    // Save to file
    std::string testFile = "/tmp/test_params.txt";
    EXPECT_TRUE(netOper.saveParametersToFile(testFile));
    
    // Create new NetOper and load
    auto netOper2 = NetOper();
    EXPECT_TRUE(netOper2.loadParametersFromFile(testFile));
    
    // Verify loaded parameters match
    auto loadedParams = netOper2.getCs();
    EXPECT_EQ(loadedParams.size(), testParams.size());
    for (size_t i = 0; i < testParams.size(); ++i) {
        EXPECT_FLOAT_EQ(loadedParams[i], testParams[i]);
    }
    
    // Cleanup
    std::remove(testFile.c_str());
}

TEST(NOP_FileIO, load_nonexistent_matrix_file) {
    auto netOper = NetOper();
    EXPECT_FALSE(netOper.loadMatrixFromFile("/nonexistent/path/matrix.txt"));
}

TEST(NOP_FileIO, load_nonexistent_params_file) {
    auto netOper = NetOper();
    EXPECT_FALSE(netOper.loadParametersFromFile("/nonexistent/path/params.txt"));
}

TEST(NOP_FileIO, save_empty_matrix) {
    auto netOper = NetOper();
    // Don't set any matrix - it should be empty
    std::string testFile = "/tmp/test_empty_matrix.txt";
    EXPECT_FALSE(netOper.saveMatrixToFile(testFile));
}

TEST(NOP_FileIO, save_empty_params) {
    auto netOper = NetOper();
    // Don't set any parameters - it should be empty
    std::string testFile = "/tmp/test_empty_params.txt";
    EXPECT_FALSE(netOper.saveParametersToFile(testFile));
}

TEST(NOP_FileIO, load_matrix_with_commas) {
    // Create a file with comma-separated values
    std::string testFile = "/tmp/test_matrix_commas.txt";
    std::ofstream file(testFile);
    file << "1, 0, 2, 0\n";
    file << "0, 1, 0, 3\n";
    file << "0, 0, 1, 0\n";
    file << "0, 0, 0, 1\n";
    file.close();
    
    auto netOper = NetOper();
    EXPECT_TRUE(netOper.loadMatrixFromFile(testFile));
    
    auto matrix = netOper.getPsi();
    EXPECT_EQ(matrix.size(), 4);
    EXPECT_EQ(matrix[0][0], 1);
    EXPECT_EQ(matrix[0][2], 2);
    EXPECT_EQ(matrix[1][3], 3);
    
    std::remove(testFile.c_str());
}

TEST(NOP_FileIO, load_params_multiline) {
    // Create a file with parameters on separate lines
    std::string testFile = "/tmp/test_params_multiline.txt";
    std::ofstream file(testFile);
    file << "1.5\n";
    file << "2.5\n";
    file << "3.5\n";
    file.close();
    
    auto netOper = NetOper();
    EXPECT_TRUE(netOper.loadParametersFromFile(testFile));
    
    auto params = netOper.getCs();
    EXPECT_EQ(params.size(), 3);
    EXPECT_FLOAT_EQ(params[0], 1.5f);
    EXPECT_FLOAT_EQ(params[1], 2.5f);
    EXPECT_FLOAT_EQ(params[2], 3.5f);
    
    std::remove(testFile.c_str());
}

// Test GenVar and Variations
TEST(NOP_Genetic, genvar_returns_valid_vector) {
    auto netOper = NetOper();
    
    // Setup a matrix for testing
    std::vector<std::vector<int>> testMatrix = {
        {1, 2, 0, 0},
        {0, 1, 3, 0},
        {0, 0, 1, 4},
        {0, 0, 0, 1}
    };
    netOper.setPsi(testMatrix);
    netOper.setNodesForVars({0});
    netOper.setNodesForParams({1});
    
    std::vector<int> w(4);
    std::mt19937 rng(42);
    EXPECT_NO_THROW(netOper.GenVar(w, rng));
    EXPECT_EQ(w.size(), 4);
    
    // w[0] should be 0-3
    EXPECT_GE(w[0], 0);
    EXPECT_LE(w[0], 3);
}

TEST(NOP_Genetic, variations_does_not_crash) {
    auto netOper = NetOper();
    
    // Setup a matrix for testing
    std::vector<std::vector<int>> testMatrix = {
        {1, 2, 0, 0},
        {0, 1, 3, 0},
        {0, 0, 1, 4},
        {0, 0, 0, 1}
    };
    netOper.setPsi(testMatrix);
    netOper.setNodesForVars({0});
    netOper.setNodesForParams({1});
    
    // Test different variation types
    std::vector<int> w1 = {0, 0, 1, 5};  // Replace non-diagonal
    EXPECT_NO_THROW(netOper.Variations(w1));
    
    std::vector<int> w2 = {1, 2, 2, 3};  // Replace diagonal
    EXPECT_NO_THROW(netOper.Variations(w2));
    
    std::vector<int> w3 = {2, 0, 2, 5};  // Add arc
    EXPECT_NO_THROW(netOper.Variations(w3));
    
    std::vector<int> w4 = {3, 0, 2, 0};  // Remove arc
    EXPECT_NO_THROW(netOper.Variations(w4));
}

TEST(NOP_Genetic, variations_empty_vector) {
    auto netOper = NetOper();
    std::vector<int> w;  // Empty vector
    EXPECT_NO_THROW(netOper.Variations(w));
}

// Test edge cases
TEST(NOP_EdgeCases, calc_result_empty_input) {
    auto netOper = NetOper();
    
    // Setup minimal configuration
    std::vector<std::vector<int>> testMatrix = {
        {1, 0},
        {0, 1}
    };
    netOper.setPsi(testMatrix);
    netOper.setNodesForVars({0});
    netOper.setNodesForParams({});
    netOper.setNodesForOutput({1});
    netOper.setCs({});
    
    std::vector<float> x_in = {0.0f};
    std::vector<float> y_out(1);
    
    EXPECT_NO_THROW(netOper.calcResult(x_in, y_out));
}

TEST(NOP_EdgeCases, calc_result_large_matrix) {
    auto netOper = NetOper();
    
    // Create a larger matrix (10x10)
    std::vector<std::vector<int>> largeMatrix(10, std::vector<int>(10, 0));
    for (int i = 0; i < 10; ++i) {
        largeMatrix[i][i] = 1;  // Diagonal
        if (i < 9) {
            largeMatrix[i][i+1] = 1;  // Some connections
        }
    }
    
    netOper.setPsi(largeMatrix);
    netOper.setNodesForVars({0, 1});
    netOper.setNodesForParams({2, 3});
    netOper.setNodesForOutput({8, 9});
    netOper.setCs({0.5f, 0.5f});
    
    std::vector<float> x_in = {1.0f, 2.0f};
    std::vector<float> y_out(2);
    
    EXPECT_NO_THROW(netOper.calcResult(x_in, y_out));
}

// Test print functions (just verify they don't crash)
TEST(NOP_Output, print_matrix_does_not_crash) {
    auto netOper = NetOper();
    netOper.setPsi(Psi);
    EXPECT_NO_THROW(netOper.printMatrix());
}

TEST(NOP_Output, get_z_returns_valid_vector) {
    auto netOper = NetOper();
    netOper.setPsi(NopPsiN);
    netOper.setNodesForVars({0, 1, 2});
    netOper.setNodesForParams({3, 4, 5});
    netOper.setNodesForOutput({22, 23});
    netOper.setCs(qc);
    
    std::vector<float> x_in = {1.0f, 2.0f, 0.5f};
    std::vector<float> y_out(2);
    netOper.calcResult(x_in, y_out);
    
    auto z = netOper.get_z();
    EXPECT_GT(z.size(), 0);
}

TEST(NOP_Output, get_parameters_returns_set_values) {
    auto netOper = NetOper();
    std::vector<float> params = {1.0f, 2.0f, 3.0f};
    netOper.setCs(params);
    
    auto retrieved = netOper.get_parameters();
    EXPECT_EQ(retrieved.size(), params.size());
    for (size_t i = 0; i < params.size(); ++i) {
        EXPECT_FLOAT_EQ(retrieved[i], params[i]);
    }
}

// Test reader integration
TEST(NOP_Reader, get_reader_returns_valid_object) {
    auto netOper = NetOper();
    NOPMatrixReader& reader = netOper.getReader();
    
    // Just verify we can get a reference to the reader
    EXPECT_NO_THROW(reader.getMatrixSize());
}

// Test boundary conditions
TEST(NOP_Boundary, operations_with_infinity) {
    auto netOper = NetOper();
    
    // Test unary operations with infinity
    EXPECT_NO_THROW(netOper.getUnaryOperationResult(1, Infinity));
    EXPECT_NO_THROW(netOper.getUnaryOperationResult(2, Infinity));
    EXPECT_NO_THROW(netOper.getUnaryOperationResult(24, Infinity));
    
    // Test binary operations with infinity
    EXPECT_NO_THROW(netOper.getBinaryOperationResult(1, Infinity, 1.0f));
    EXPECT_NO_THROW(netOper.getBinaryOperationResult(2, Infinity, 1.0f));
}

TEST(NOP_Boundary, operations_with_zero) {
    auto netOper = NetOper();
    
    // Test all unary operations with zero
    for (int i = 1; i <= 28; ++i) {
        EXPECT_NO_THROW(netOper.getUnaryOperationResult(i, 0.0f));
    }
    
    // Test all binary operations with zero
    for (int i = 1; i <= 8; ++i) {
        EXPECT_NO_THROW(netOper.getBinaryOperationResult(i, 0.0f, 0.0f));
    }
}

TEST(NOP_Boundary, operations_with_negative_values) {
    auto netOper = NetOper();
    
    // Test operations with negative values
    EXPECT_NO_THROW(netOper.getUnaryOperationResult(1, -5.0f));
    EXPECT_NO_THROW(netOper.getUnaryOperationResult(3, -5.0f));
    EXPECT_NO_THROW(netOper.getUnaryOperationResult(11, -5.0f));
    
    EXPECT_NO_THROW(netOper.getBinaryOperationResult(1, -5.0f, -3.0f));
    EXPECT_NO_THROW(netOper.getBinaryOperationResult(2, -5.0f, -3.0f));
}
