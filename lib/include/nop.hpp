#pragma once

#include "baseFunctions.hpp"
#include "reader.h"
#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>


// class network operator
class NetOper
{
public:

    // default constructor
    NetOper();

    // RPCntrol
    void calcResult(const std::vector<float>& x_in, std::vector<float>& y_out);

    float getUnaryOperationResult(int operationNum, float input);
    float getBinaryOperationResult(int operationNum, float left, float right);
    
    // TODO use move semantics and rvalues
    void setNodesForVars(const std::vector<int>& nodes);
    const std::vector<int>& getNodesForVars();

    void setNodesForParams(const std::vector<int>& nodes);
    const std::vector<int>& getNodesForParams();

    void setNodesForOutput(const std::vector<int>& nodes);
    const std::vector<int>& getNodesForOutput();

    void setCs(const std::vector<float>& newParams);
    const std::vector<float>& getCs();

    void setPsi(const std::vector<std::vector<int>>& newMatrix);
    const std::vector<std::vector<int>>& getPsi();

    NOPMatrixReader& getReader();
    
    void setLocalTestsParameters();
    void printMatrix() const;

    /**
     * @brief Загрузить матрицу из текстового файла
     * 
     * Формат файла: числа разделены пробелами/запятыми, каждая строка - новая строка матрицы
     * Пример:
     *   1 0 0 0 0 0 1 10 0 0 12 1 ...
     *   0 1 0 0 0 0 0 1 0 0 0 0 ...
     *   ...
     * 
     * @param filepath Путь к файлу матрицы
     * @return true если успешно загружено, false если ошибка
     */
    bool loadMatrixFromFile(const std::string& filepath);
    
    /**
     * @brief Загрузить параметры из текстового файла
     * 
     * Формат файла: числа разделены пробелами или на отдельных строках
     * Пример: 41974.2 29423.1 53775.6 16406.0 41974.2 29423.1 53775.6 16406.0
     * 
     * @param filepath Путь к файлу параметров
     * @return true если успешно загружено, false если ошибка
     */
    bool loadParametersFromFile(const std::string& filepath);

    /**
     * @brief Сохранить матрицу в текстовый файл
     * 
     * Формат: числа разделены пробелами, строки - на новых строках
     * 
     * @param filepath Путь к файлу для сохранения
     * @return true если успешно сохранено, false если ошибка
     */
    bool saveMatrixToFile(const std::string& filepath) const;

    /**
     * @brief Сохранить параметры в текстовый файл
     * 
     * Формат: числа разделены пробелами на одной строке
     * 
     * @param filepath Путь к файлу для сохранения
     * @return true если успешно сохранено, false если ошибка
     */
    bool saveParametersToFile(const std::string& filepath) const;


    void GenVar(std::vector<int>& w, std::mt19937& rng);
    void Variations(const std::vector<int>& w);

    std::vector<float>& get_z();
    std::vector<float>& get_parameters();


private:
    void initUnaryFunctionsMap();
    void initBinaryFunctionsMap();
    bool TestSource(int j);


private:
    NOPMatrixReader m_reader;
    size_t m_numOutputs;                   // Mout
    std::vector<float> m_parameters;       // Cs
    std::vector<int> m_nodesForVars;       // Pnum
    std::vector<int> m_nodesForParams;     // Rnum
    std::vector<int> m_nodesForOutput;     // Dnum
    std::vector<float> z;                  // z

    std::vector<std::vector<int>> m_matrix; // Psi

    std::map<int, float(*)(float)> m_unaryFuncMap;
    std::map<int, float(*)(float, float)> m_binaryFuncMap;
};


extern const std::vector<std::vector<int>> NopPsiN;
extern std::vector<float> qc;
extern const std::vector<std::vector<int>> Psi;
