/**
 * @file BaseSolution.hpp
 * @brief Базовый класс для решений с сетевым оператором
 */

#pragma once
#include "isolution.hpp"
#include "base_config.hpp"
#include "nop.hpp"
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <random>


/**
 * @class BaseSolution
 * @brief Базовый класс для всех решений, использующих NetOper
 * 
 * Инкапсулирует общую логику:
 * - Инициализацию сетевого оператора из конфига
 * - Декодирование хромосом
 * - Преобразование кода Грея в параметры
 * 
 * Количество параметров берётся из конфига (size of base_params)
 * Биты (int_bits, frac_bits) устанавливаются снаружи через setters
 */
template<typename ConfigType>
class BaseSolution : public ISolution {
public:
    /**
     * @brief Конструктор
     * @param config Конфигурация (SimpleConfig или RobotProblemConfig)
     * 
     * Количество параметров определяется автоматически из config.base_params.size()
     * Биты остаются неинициализированными - должны быть установлены через setIntBits/setFracBits
     */
    explicit BaseSolution(const ConfigType& config)
        : config_(config), 
          int_bits_(0), 
          frac_bits_(0), 
          num_params_(static_cast<int>(config_.base_params.size())) {
        
        // Инициализируем NetOper из конфига
        initializeNetOper();
    }
    
    virtual ~BaseSolution() = default;
    
    
    // ===== РЕАЛИЗАЦИЯ ISolution =====
    
    /**
     * @brief Декодирование хромосомы в параметры
     */
    void decode(const std::vector<int>& chromosome_params,
                const std::vector<std::vector<int>>& chromosome_struct) override {
        try {        
            net_oper_.setPsi(config_.base_matrix);
            
            for (size_t i = 0; i < chromosome_struct.size(); ++i) {
                net_oper_.Variations(chromosome_struct[i]);
            }
            
            greyToVector(chromosome_params);
            
        } catch (const std::exception& e) {
            std::cerr << "Error decoding solution: " << e.what() << std::endl;
            throw;
        }
    }
    
    
    /**
     * @brief Клонирование решения
     */
    std::unique_ptr<ISolution> clone() const override {
        auto cloned = std::make_unique<BaseSolution<ConfigType>>(config_);
        cloned->setIntBits(int_bits_);
        cloned->setFracBits(frac_bits_);
        return cloned;
    }
    
    
    /**
     * @brief Получение параметров
     */
    std::vector<float> getParameters() const override {
        auto& net = const_cast<NetOper&>(net_oper_);
        auto params = net.get_parameters();
        
        if (params.empty()) {
            std::cerr << "Warning: NetOper parameters are empty" << std::endl;
        }
        
        return params;
    }
    
    
    // ===== ДОСТУП К NetOper =====
    
    /// Получить неконстантную ссылку на NetOper
    NetOper& getNetOper() { 
        return net_oper_; 
    }
    
    /// Получить константную ссылку на NetOper
    const NetOper& getNetOperConst() const { 
        return net_oper_; 
    }
    
    
    // ===== ДОСТУП К КОНФИГУ =====
    
    /// Получить копию конфигурации
    ConfigType getConfig() const {
        return config_;
    }
    
    /// Получить ссылку на конфигурацию
    const ConfigType& getConfigRef() const {
        return config_;
    }
    
    
    // ===== ГЕНЕРИРОВАНИЕ ВАРИАЦИЙ =====
    
    /// Генерировать вариацию (вызывает NOP.GenVar)
    void generateVariation(std::vector<int>& variation, std::mt19937& rng) {
        net_oper_.GenVar(variation, rng);
    }
    
    
    // ===== НАСТРОЙКА БИТОВ =====
    
    /// Установить количество целых битов
    void setIntBits(int bits) { 
        int_bits_ = bits; 
    }
    
    /// Установить количество дробных битов
    void setFracBits(int bits) { 
        frac_bits_ = bits; 
    }
    
    /// Получить количество целых битов
    int getIntBits() const { 
        return int_bits_; 
    }
    
    /// Получить количество дробных битов
    int getFracBits() const { 
        return frac_bits_; 
    }
    
    /// Получить количество параметров (из конфига)
    int getNumParams() const {
        return num_params_;
    }
    
    
protected:
    NetOper net_oper_;
    ConfigType config_;
    int int_bits_;
    int frac_bits_;
    int num_params_;
    
    
    /**
     * @brief Инициализация NetOper из конфигурации
     */
    void initializeNetOper() {
        net_oper_.setNodesForVars(config_.nodes_for_vars);
        net_oper_.setNodesForParams(config_.nodes_for_params);
        net_oper_.setNodesForOutput(config_.nodes_for_output);
        net_oper_.setCs(config_.base_params);
        net_oper_.setPsi(config_.base_matrix);
    }
    
    
    /**
     * @brief Преобразование кода Грея в вектор параметров
     * 
     * Алгоритм:
     * 1. Преобразование кода Грея в бинарный код
     * 2. Группировка битов по параметрам (int_bits + frac_bits каждый)
     * 3. Преобразование битов в числа с фиксированной точкой
     */
    void greyToVector(const std::vector<int>& grey_code) {
        try {
            if (grey_code.empty()) {
                std::cerr << "Warning: grey_code is empty" << std::endl;
                return;
            }

            std::vector<int> binary_code(grey_code.size(), 0);
            // Layout per param: [1 sign bit | int_bits | frac_bits]
            int bits_per_param = 1 + int_bits_ + frac_bits_;

            // Grey -> Binary
            for (size_t i = 0; i < grey_code.size(); ++i) {
                int base = static_cast<int>(i) - (static_cast<int>(i) % bits_per_param);
                if (static_cast<int>(i) == base) {
                    binary_code[i] = grey_code[i];
                } else {
                    binary_code[i] = binary_code[i - 1] ^ grey_code[i];
                }
            }

            // Binary -> Float
            auto& params = const_cast<NetOper&>(net_oper_).get_parameters();
            params.clear();

            double g1 = std::pow(2.0, int_bits_ - 1);

            for (int param_idx = 0; param_idx < num_params_; ++param_idx) {
                int base = param_idx * bits_per_param;

                if (base + bits_per_param > static_cast<int>(binary_code.size())) {
                    break;
                }

                // Sign bit
                bool negative = (binary_code[base] == 1);

                double value = 0.0;
                double g = g1;

                // Integer part (after sign bit)
                int int_start = base + 1;
                int int_end = int_start + int_bits_;
                for (int i = int_start; i < int_end; ++i) {
                    value += g * binary_code[i];
                    g /= 2.0;
                }

                // Fractional part
                int frac_end = base + bits_per_param;
                for (int i = int_end; i < frac_end; ++i) {
                    value += g * binary_code[i];
                    g /= 2.0;
                }

                if (negative) value = -value;
                params.push_back(static_cast<float>(value));
            }

        } catch (const std::exception& e) {
            std::cerr << "Error in greyToVector: " << e.what() << std::endl;
            throw;
        }
    }
};
