#include "nop.hpp"
#include <iostream>
#include <random>


NetOper::NetOper()
{
    initUnaryFunctionsMap();
    initBinaryFunctionsMap();
}

float NetOper::getUnaryOperationResult(int operationNum, float input)
{
    return m_unaryFuncMap[operationNum](input);
}

float NetOper::getBinaryOperationResult(int operationNum, float left, float right)
{
    return m_binaryFuncMap[operationNum](left, right);
}

void NetOper::initUnaryFunctionsMap()
{
    m_unaryFuncMap[1] = ro_1;
    m_unaryFuncMap[2] = ro_2;
    m_unaryFuncMap[3] = ro_3;
    m_unaryFuncMap[4] = ro_4;
    m_unaryFuncMap[5] = ro_5;
    m_unaryFuncMap[6] = ro_6;
    m_unaryFuncMap[7] = ro_7;
    m_unaryFuncMap[8] = ro_8;
    m_unaryFuncMap[9] = ro_9;
    m_unaryFuncMap[10] = ro_10;
    m_unaryFuncMap[11] = ro_11;
    m_unaryFuncMap[12] = ro_12;
    m_unaryFuncMap[13] = ro_13;
    m_unaryFuncMap[14] = ro_14;
    m_unaryFuncMap[15] = ro_15;
    m_unaryFuncMap[16] = ro_16;
    m_unaryFuncMap[17] = ro_17;
    m_unaryFuncMap[18] = ro_18;
    m_unaryFuncMap[19] = ro_19;
    m_unaryFuncMap[20] = ro_20;
    m_unaryFuncMap[21] = ro_21;
    m_unaryFuncMap[22] = ro_22;
    m_unaryFuncMap[23] = ro_23;
    m_unaryFuncMap[24] = ro_24;
    m_unaryFuncMap[25] = ro_25;
    m_unaryFuncMap[26] = ro_26;
    m_unaryFuncMap[27] = ro_27;
    m_unaryFuncMap[28] = ro_28;
}

void NetOper::initBinaryFunctionsMap()
{
    m_binaryFuncMap[1] = xi_1;
    m_binaryFuncMap[2] = xi_2;
    m_binaryFuncMap[3] = xi_3;
    m_binaryFuncMap[4] = xi_4;
    m_binaryFuncMap[5] = xi_5;
    m_binaryFuncMap[6] = xi_6;
    m_binaryFuncMap[7] = xi_7;
    m_binaryFuncMap[8] = xi_8;
}

const std::vector<int>& NetOper::getNodesForVars()
{
    return m_nodesForVars;
}

void NetOper::setNodesForVars(const std::vector<int>& nodes)
{
    m_nodesForVars = nodes;
}

const std::vector<int>& NetOper::getNodesForParams()
{
    return m_nodesForParams;
}

void NetOper::setNodesForParams(const std::vector<int>& nodes)
{
    m_nodesForParams = nodes;
}

const std::vector<int>& NetOper::getNodesForOutput()
{
    return m_nodesForOutput;
}

void NetOper::setNodesForOutput(const std::vector<int>& nodes)
{
    m_nodesForOutput = nodes;
}

const std::vector<std::vector<int>>& NetOper::getPsi()
{
    return m_matrix;
}

const std::vector<float>& NetOper::getCs()
{
    return m_parameters;
}

void NetOper::setCs(const std::vector<float>& newParams)
{
    m_parameters = newParams;
}

void NetOper::setPsi(const std::vector<std::vector<int>>& newMatrix)
{
    m_matrix = newMatrix;
    z.resize(m_matrix.size());
}
// ROControl
void NetOper::calcResult(const std::vector<float>& x_in, std::vector<float>& y_out)
{
    // Large finite values instead of ±Infinity.
    // Infinity propagates through +,-,* and corrupts outputs.
    // ±1e30 is "large enough" for max/min identity, but finite.
    constexpr float BigVal = 1e30f;

    for(size_t i=0; i < m_matrix.size(); ++i)
    {
        if (m_matrix[i][i] == 2)
            z[i] = 1.0f;
        else if (m_matrix[i][i] == 3)
            z[i] = -BigVal;
        else if (m_matrix[i][i] == 4)
            z[i] = BigVal;
        else
            z[i] = 0.0f;
    }

    for(size_t i=0; i < m_nodesForVars.size(); ++i)
    {
        z[m_nodesForVars[i]] = x_in[i];
    }
    for (size_t i=0; i < m_nodesForParams.size(); ++i)
    {
        z[m_nodesForParams[i]] = m_parameters[i];
    }
    for(size_t i=0; i < m_matrix.size() - 1; ++i)
    {
        for(size_t j=i+1; j < m_matrix.size(); ++j)
        {
            if (m_matrix[i][j] == 0)
                continue;

            auto zz = getUnaryOperationResult(m_matrix[i][j], z[i]);
            z[j] = getBinaryOperationResult(m_matrix[j][j], z[j], zz);

            // Clamp non-finite values (NaN from overflow like Infinity*0).
            // Don't clamp finite values — let them propagate naturally.
            if (!std::isfinite(z[j]))
                z[j] = 0.0f;
        }
    }
    for(size_t i = 0; i < m_nodesForOutput.size(); ++i)
        y_out[i] = z[m_nodesForOutput[i]];
    
}

NOPMatrixReader& NetOper::getReader()
{
    return m_reader;
}

// todo update
std::vector<float>& NetOper::get_z()
{
    return z;
}

std::vector<float>& NetOper::get_parameters()
{
    return m_parameters;
}

bool NetOper::TestSource(int j)
{
    for (int node : m_nodesForVars)
        if (j == node) return false;

    for (int node : m_nodesForParams)
        if (j == node) return false;

    return true;
}

void NetOper::GenVar(std::vector<int>& w, std::mt19937& rng)
{
    if (w.size() < 4) w.resize(4);

    int L = static_cast<int>(m_matrix.size());
    int kW = static_cast<int>(m_unaryFuncMap.size());
    int kV = static_cast<int>(m_binaryFuncMap.size());

    std::uniform_int_distribution<int> dist_op(0, 3);
    std::uniform_int_distribution<int> dist_unary(1, kW);   // [1, kW] inclusive
    std::uniform_int_distribution<int> dist_binary(1, kV);  // [1, kV] inclusive

    w[0] = dist_op(rng);

    switch (w[0])
    {
    case 0:
    case 2:
    case 3: {
        std::uniform_int_distribution<int> dist_i(0, L - 2);
        w[1] = dist_i(rng);
        std::uniform_int_distribution<int> dist_j(w[1] + 1, L - 1);
        w[2] = dist_j(rng);
        w[3] = dist_unary(rng);
        break;
    }

    case 1: {
        std::uniform_int_distribution<int> dist_node(0, L - 1);
        w[1] = dist_node(rng);

        int start = w[1];
        bool found = false;
        for (int i = 0; i < L; ++i) {
            int idx = (start + i) % L;
            if (TestSource(idx)) {
                w[1] = idx;
                found = true;
                break;
            }
        }
        if (!found) w[1] = 0;

        w[2] = w[1];
        w[3] = dist_binary(rng);
        break;
    }
    }
}

// приминение вариации
void NetOper::Variations(const std::vector<int>& w)
{
    if (w.size() < 4) return; // safety

    if (w[0] != 0 || w[1] != 0 || w[2] != 0)
    {
        switch (w[0])
        {
        case 0: // замена недиагонального элемента
            if (m_matrix[w[1]][w[2]] != 0)
                m_matrix[w[1]][w[2]] = w[3];
            break;

        case 1: // замена диагонального элемента
            if (m_matrix[w[1]][w[1]] != 0)
                m_matrix[w[1]][w[1]] = w[3];
            break;

        case 2: // добавление дуги
            if (m_matrix[w[1]][w[2]] == 0)
                if (m_matrix[w[2]][w[2]] != 0)
                    m_matrix[w[1]][w[2]] = w[3];
            break;

        case 3: // удаление дуги
        {
            int s1 = 0;
            for (int i = 0; i < w[2]; i++)
            {
                if (m_matrix[i][w[2]] != 0)
                    s1++;
            }

            int s2 = 0;
            for (int j = w[1] + 1; j < static_cast<int>(m_matrix.size()); j++)
            {
                if (m_matrix[w[1]][j] != 0)
                    s2++;
            }

            if (s1 > 1 && s2 > 1)
                m_matrix[w[1]][w[2]] = 0;
            break;
        }
        }
    }
}

void NetOper::printMatrix() const
{
    for (const auto& row : m_matrix)
    {
        for (auto val : row)
            std::cout << val << " ";
        std::cout << "\n";
    }
    std::cout << "---------------------\n";
}

// only for local tests
void NetOper::setLocalTestsParameters()
{
    NOPMatrixReader& reader = this->getReader();
    reader.readMatrix("/home/user/catkin_ws/src/rosbot_nop_controller/data/24_NOP_461");
    reader.readParams("/home/user/catkin_ws/src/rosbot_nop_controller/data/q_461.txt");

    this->setNodesForVars({0, 1, 2});   // Pnum
    this->setNodesForParams({3, 4, 5}); // Rnum
    this->setNodesForOutput({22, 23});  // Dnum
    this->setCs(reader.getParams());
    this->setPsi(reader.getMatrix());
}


/**
 * @brief Загрузить матрицу из текстового файла
 * 
 * Формат: числа разделены пробелами, строки - на новых строках
 * Пример содержимого файла matrix.txt:
 *   1 0 0 0 0 0 1 10 0 0 12 1 ...
 *   0 1 0 0 0 0 0 1 0 0 0 0 ...
 *   ...
 */
bool NetOper::loadMatrixFromFile(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open matrix file: " << filepath << std::endl;
        return false;
    }

    m_matrix.clear();
    std::string line;
    int row_count = 0;

    while (std::getline(file, line)) {
        // Пропускаем пустые строки
        if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos) {
            continue;
        }

        std::vector<int> row;
        std::stringstream ss(line);
        int value;

        // Парсим числа из строки (разделители: пробел, запятая)
        while (ss >> value) {
            row.push_back(value);
            // Пропускаем запятые если есть
            if (ss.peek() == ',') {
                ss.ignore();
            }
        }

        if (row.empty()) {
            std::cerr << "WARNING: Empty row at line " << (row_count + 1) << std::endl;
            continue;
        }

        m_matrix.push_back(row);
        row_count++;
    }

    file.close();

    if (m_matrix.empty()) {
        std::cerr << "ERROR: Matrix is empty after loading from " << filepath << std::endl;
        return false;
    }

    // Проверяем что матрица квадратная и все строки одинакового размера
    size_t expected_cols = m_matrix[0].size();
    for (size_t i = 0; i < m_matrix.size(); ++i) {
        if (m_matrix[i].size() != expected_cols) {
            std::cerr << "ERROR: Row " << i << " has " << m_matrix[i].size() 
                      << " columns, expected " << expected_cols << std::endl;
            return false;
        }
    }

    // Проверяем что матрица квадратная (для NOP это требование)
    if (m_matrix.size() != expected_cols) {
        std::cerr << "WARNING: Matrix is not square! Rows: " << m_matrix.size() 
                  << ", Cols: " << expected_cols << std::endl;
        // Не критично, но важно знать
    }

    // Инициализируем z с нужным размером
    z.resize(m_matrix.size());

    std::cout << "✓ Successfully loaded matrix from " << filepath << std::endl;
    std::cout << "  Matrix size: " << m_matrix.size() << " x " << expected_cols << std::endl;

    return true;
}


/**
 * @brief Загрузить параметры из текстового файла
 * 
 * Формат: числа разделены пробелами или на отдельных строках
 * Пример содержимого файла params.txt:
 *   41974.2 29423.1 53775.6 16406.0 41974.2 29423.1 53775.6 16406.0
 * 
 * Или каждое число на отдельной строке:
 *   41974.2
 *   29423.1
 *   53775.6
 *   16406.0
 */
bool NetOper::loadParametersFromFile(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open parameters file: " << filepath << std::endl;
        return false;
    }

    m_parameters.clear();
    std::string line;
    float value;

    // Читаем весь файл как поток значений (пробельные разделители)
    while (file >> value) {
        m_parameters.push_back(value);
    }

    file.close();

    if (m_parameters.empty()) {
        std::cerr << "ERROR: No parameters loaded from " << filepath << std::endl;
        return false;
    }

    std::cout << "✓ Successfully loaded " << m_parameters.size() << " parameters from " << filepath << std::endl;
    std::cout << "  Parameters: ";
    for (size_t i = 0; i < m_parameters.size(); ++i) {
        std::cout << m_parameters[i];
        if (i < m_parameters.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    return true;
}


/**
 * @brief Сохранить матрицу в текстовый файл
 * 
 * Формат: числа разделены пробелами, строки - на новых строках
 * Пример вывода:
 *   1 0 0 0 0 0 1 10 0 0 12 1 ...
 *   0 1 0 0 0 0 0 1 0 0 0 0 ...
 *   ...
 */
bool NetOper::saveMatrixToFile(const std::string& filepath) const
{
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open matrix file for writing: " << filepath << std::endl;
        return false;
    }

    if (m_matrix.empty()) {
        std::cerr << "ERROR: Matrix is empty, nothing to save!" << std::endl;
        file.close();
        return false;
    }

    // Записываем каждую строку матрицы
    for (size_t i = 0; i < m_matrix.size(); ++i) {
        for (size_t j = 0; j < m_matrix[i].size(); ++j) {
            file << m_matrix[i][j];
            if (j < m_matrix[i].size() - 1) {
                file << " ";  // Разделитель между элементами
            }
        }
        file << "\n";  // Новая строка после каждой строки матрицы
    }

    file.close();

    std::cout << "✓ Successfully saved matrix to " << filepath << std::endl;
    std::cout << "  Matrix size: " << m_matrix.size() << " x " << m_matrix[0].size() << std::endl;

    return true;
}


/**
 * @brief Сохранить параметры в текстовый файл
 * 
 * Формат: числа разделены пробелами на одной строке
 * Пример вывода: 41974.2 29423.1 53775.6 16406.0 41974.2 29423.1 53775.6 16406.0
 */
bool NetOper::saveParametersToFile(const std::string& filepath) const
{
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open parameters file for writing: " << filepath << std::endl;
        return false;
    }

    if (m_parameters.empty()) {
        std::cerr << "ERROR: Parameters are empty, nothing to save!" << std::endl;
        file.close();
        return false;
    }

    // Записываем параметры в одну строку, разделённые пробелами
    for (size_t i = 0; i < m_parameters.size(); ++i) {
        file << m_parameters[i];
        if (i < m_parameters.size() - 1) {
            file << " ";  // Разделитель между параметрами
        }
    }
    file << "\n";  // Новая строка в конце

    file.close();

    std::cout << "✓ Successfully saved " << m_parameters.size() << " parameters to " << filepath << std::endl;
    std::cout << "  Parameters: ";
    for (size_t i = 0; i < m_parameters.size(); ++i) {
        std::cout << m_parameters[i];
        if (i < m_parameters.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    return true;
}

const std::vector<std::vector<int>> NopPsiN = {
    {1, 0, 0, 0, 0, 0, 1, 10, 0, 0, 12, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10},
    {0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 19, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 2, 9, 0, 0, 0, 0, 10, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 6, 0, 0, 0, 0, 13, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 22, 0, 0, 0, 0, 11, 0, 0, 0, 0, 0, 0, 19},
    {0, 0, 1, 0, 0, 0, 6, 0, 0, 8, 0, 5, 0, 4, 13, 10, 0, 0, 0, 14, 15, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 10, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 8, 0, 0, 0, 12, 0, 0, 1, 19, 0, 0, 0},
    {0, 1, 0, 0, 0, 2, 0, 0, 0, 1, 1, 8, 0, 0, 0, 1, 8, 0, 0, 0, 14, 12, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 1, 26, 5, 4, 23, 0, 0, 0, 0, 15, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 17, 10, 10, 0, 0, 0, 0, 16, 0, 16, 0, 16},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 14, 0, 25, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 10, 0, 1, 0, 0, 12, 0, 13},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 8, 0, 0, 27, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 1, 0, 15, 4, 0, 0},
    {0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0, 0, 1, 0, 0, 0, 7, 0, 17, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 21, 0, 16, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 1, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 0, 0, 0, 0, 0, 7, 16, 13},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6}
};




std::vector<float> qc = {33879.9, 43369.9, 52114.8, 27901.6};


// Base NetOper matrix to represent simple desiredFucntion from nop_test and controller_test
// for simple sin/cos task
// std::vector<float> qc = {1.0, 1.0, 1.0};
// optimal for simple
const std::vector<std::vector<int>> Psi = {
    {0, 0, 0, 0, 0, 1, 1, 26, 0, 2, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 4, 0, 0, 3, 0, 22, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 12},
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 16, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 8, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 14, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 11, 8},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 13},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 18},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}
};