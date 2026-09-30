# Руководство по использованию генетического алгоритма оптимизации

## 📋 Оглавление
1. [Обзор](#обзор)
2. [Структура проекта](#структура-проекта)
3. [Конфигурация](#конфигурация)
4. [Параметры генетического алгоритма](#параметры-генетического-алгоритма)
5. [Сохранение и загрузка результатов](#сохранение-и-загрузка-результатов)
6. [Запуск оптимизации](#запуск-оптимизации)
7. [Результаты](#результаты)

---

## Обзор

Данный проект реализует **генетический алгоритм (GA)** для оптимизации **сетевого оператора (NetOper)** на примере задачи управления мобильным роботом. 

**Основные компоненты:**
- `NetOper` - сетевой оператор с матрицей и параметрами
- `GANOP` - генетический алгоритм оптимизации
- `RobotFitnessEvaluator` - функция приспособленности (fitness) для робота
- `RobotProblemConfig` - конфигурация задачи оптимизации
- `GAConfig` - конфигурация параметров генетического алгоритма

---

## Структура проекта

```
rosbot_nop_controller/
├── include/
│   ├── net_oper_my/
│   │   ├── nop.hpp              # Класс NetOper с методами load/save
│   │   └── app/
│   │       └── train_robot_control.cpp  # Главный файл оптимизации
│   ├── GANOP.hpp                # Генетический алгоритм
│   ├── RobotFitnessEvaluator.hpp # Оценка приспособленности
│   ├── RobotProblemConfig.hpp    # Конфигурация задачи
│   └── controller.hpp            # Контроллер робота
├── data/
│   ├── best_matrix.txt          # 📁 Сохранённая матрица (создаётся после оптимизации)
│   ├── best_params.txt          # 📁 Сохранённые параметры (создаётся после оптимизации)
│   ├── matrix.txt               # 📁 Исходная матрица (опционально)
│   └── params.txt               # 📁 Исходные параметры (опционально)
├── CMakeLists.txt
└── README.md
```

---

## Конфигурация

### 1. RobotProblemConfig - конфигурация задачи

Файл: `include/RobotProblemConfig.hpp`

```cpp
RobotProblemConfig robot_config;

// Параметры симуляции
robot_config.dt = 0.033333f;              // Шаг симуляции (33ms)
robot_config.time_limit = 30.0f;          // Максимальное время траектории
robot_config.epsilon_term = 0.1f;         // Расстояние до цели для остановки

// Траектории
robot_config.num_trajectories = 64;       // Количество траекторий для обучения GA
robot_config.num_test_trajectories = 64;  // Количество траекторий для тестирования

// Границы начальных состояний
robot_config.qyminc = {-5.5f, -5.5f, -1.31f};   // Min: x, y, theta
robot_config.qymaxc = {5.5f, 5.5f, 1.31f};     // Max: x, y, theta

// Путь к ONNX модели
robot_config.model_path = "rosbot_gazebo9_2d_model.onnx";

// Структура сети (узлы для переменных, параметров, выходов)
robot_config.nodes_for_vars = {0, 1, 2};              // Входные узлы (x, y, theta)
robot_config.nodes_for_params = {3, 4, 5, 6, 7, 8, 9, 10};  // Узлы параметров (8 шт)
robot_config.nodes_for_output = {30, 31};             // Выходные узлы (два управления)

// Матрица и параметры сети
robot_config.base_matrix = {...};        // Матрица связей сети (32x32)
robot_config.base_params = {...};        // Базовые 8 параметров
```

**Где эти значения используются:**
- `num_trajectories` - GA генерирует это количество начальных состояний для обучения
- `num_test_trajectories` - после оптимизации симулируются эти траектории для тестирования
- `nodes_for_params` - указывает какие узлы сети будут содержать оптимизируемые параметры (всего 8)
- `base_matrix` - исходная архитектура сети, которая будет эволюционировать

### 2. GAConfig - конфигурация генетического алгоритма

Файл: `train_robot_control.cpp` в `main()`

```cpp
GAConfig ga_config;

// Структура сети (скопируется из RobotProblemConfig)
ga_config.nodes_for_vars = robot_config.nodes_for_vars;
ga_config.nodes_for_params = robot_config.nodes_for_params;
ga_config.nodes_for_output = robot_config.nodes_for_output;

// ===== ПАРАМЕТРЫ ГЕНЕТИЧЕСКОГО АЛГОРИТМА =====

// Параметры популяции
ga_config.population_size = 500;          // Размер популяции на каждом поколении
ga_config.num_generations = 32;           // Количество поколений для эволюции

// Кроссовер и мутация
ga_config.num_crossovers_per_gen = 24;    // Количество операций скрещивания за поколение
ga_config.mutation_prob = 0.5f;           // Вероятность мутации (50%)

// Селекция
ga_config.selection_alpha = 0.5f;         // Параметр селекции (от 0 до 1)
ga_config.search_neighbors = 64;          // Количество соседей для поиска

// Представление решений
ga_config.int_bits = 16;                  // Целая часть числа (16 бит)
ga_config.frac_bits = 16;                 // Дробная часть числа (16 бит)

// Параметры сети
ga_config.num_params = 8;                 // Количество оптимизируемых параметров

// Мутация структуры сети
ga_config.num_struct_variations = 15;     // Количество вариаций структуры за итерацию

// Воспроизводимость
ga_config.seed = 69;                      // Seed для генератора случайных чисел
```

**Что означает каждый параметр:**
- `population_size` - больше = медленнее но лучше, меньше = быстрее но хуже
- `num_generations` - больше = лучше но дольше
- `num_crossovers_per_gen` - частота скрещивания в каждом поколении
- `mutation_prob` - 0.5 = 50% вероятность мутации каждого гена
- `int_bits + frac_bits` - точность представления параметров (16+16 = 32-bit float)

---

## Параметры генетического алгоритма

### Рекомендуемые конфигурации

#### 🚀 Быстрая оптимизация (1-5 минут)
```cpp
ga_config.population_size = 100;
ga_config.num_generations = 10;
ga_config.num_crossovers_per_gen = 10;
robot_config.num_trajectories = 16;
```

#### ⚖️ Средняя оптимизация (20-30 миннут)
```cpp
ga_config.population_size = 300;
ga_config.num_generations = 20;
ga_config.num_crossovers_per_gen = 18;
robot_config.num_trajectories = 32;
```

#### 🎯 Глубокая оптимизация (1-2 часа)
```cpp
ga_config.population_size = 500;
ga_config.num_generations = 32;
ga_config.num_crossovers_per_gen = 24;
robot_config.num_trajectories = 64;
```

---

## Сохранение и загрузка результатов

### Структура методов в NetOper

**Сохранение:**
```cpp
net.saveMatrixToFile("best_matrix.txt");      // Сохраняет матрицу
net.saveParametersToFile("best_params.txt");  // Сохраняет параметры
```

**Загрузка:**
```cpp
net.loadMatrixFromFile("best_matrix.txt");      // Загружает матрицу из файла
net.loadParametersFromFile("best_params.txt");  // Загружает параметры из файла
```

### Форматы файлов

**best_matrix.txt** - матрица 32x32, числа разделены пробелами:
```
1 0 0 0 0 0 1 10 0 0 12 1 0 0 0 0 0 0 0 0 0 0 0 10 0 0 0 0 0 0 0 0
0 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 19 0 0 0 0 0 0 0 0 0 0 0 0
...
```

**best_params.txt** - 8 параметров через пробел:
```
41974.2 29423.1 53775.6 16406.0 41974.2 29423.1 53775.6 16406.0
```

### Автоматическое сохранение и загрузка

В `main()` реализована логика:

```cpp
// При запуске проверяются файлы
if (file_exists("best_matrix.txt") && file_exists("best_params.txt")) {
    // Загружаются сохранённые результаты
    ga_config.nop_template->loadMatrixFromFile("best_matrix.txt");
    ga_config.nop_template->loadParametersFromFile("best_params.txt");
} else {
    // Используются базовые конфиги
    ga_config.nop_template->setCs(robot_config.base_params);
    ga_config.nop_template->setPsi(robot_config.base_matrix);
}

// После оптимизации автоматически сохраняются
ga_config.on_algorithm_end = save_best_solution;  // Вызывает saveMatrixToFile и saveParametersToFile
```

---

## Запуск оптимизации

### Полный workflow

#### Шаг 1: Отредактируй параметры в `train_robot_control.cpp`

```cpp
// В main() изменяй эти значения:
robot_config.num_trajectories = 64;           // ← Управляет сложностью
robot_config.num_test_trajectories = 64;      // ← Для тестирования

ga_config.population_size = 500;              // ← Размер популяции
ga_config.num_generations = 32;               // ← Количество поколений
ga_config.num_crossovers_per_gen = 24;        // ← Операции скрещивания
```

#### Шаг 2: Построй проект

```bash
cd /path/to/rosbot_nop_controller
mkdir -p build && cd build
cmake ..
make  # или make -j4 для параллельной сборки
```

#### Шаг 3: Запусти оптимизацию

```bash
./train в зависимости от конфигурации CMake
```

#### Шаг 4: Отслеживай прогресс

Программа выведет:
```
=== LOADING NETWORK STATE ===
No saved network state found, using base configuration
✓ Base network configuration loaded
NetOper template initialized

=== STARTING GENETIC ALGORITHM ===
Population: 500
Generations: 32
Training trajectories: 64
Test trajectories: 64

Gen 0 done, avg: 123.45
Gen 1 done, avg: 125.67
Gen 2 done, avg: 128.90
...
Gen 31 done, avg: 145.23

=== GA COMPLETED SUCCESSFULLY ===
Results saved to:
  - best_matrix.txt
  - best_params.txt
  - trajectories.csv
  - evolution_log.txt
```

---

## Результаты

### Создаваемые файлы

После завершения оптимизации в текущей директории появляются:

| Файл | Содержимое | Использование |
|------|-----------|---------------|
| `best_matrix.txt` | Оптимизированная матрица 32x32 | При следующем запуске автоматически загружается |
| `best_params.txt` | 8 оптимизированных параметров | При следующем запуске автоматически загружается |
| `trajectories.csv` | Симуляция 64 траекторий робота | Анализ поведения, визуализация в matplotlib |
| `evolution_log.txt` | История приспособленности | Анализ сходимости алгоритма |

### Анализ результатов

#### 1. Просмотр истории эволюции
```bash
cat evolution_log.txt
```

Пример вывода:
```
Generation 0: avg_fitness = 123.45
Generation 1: avg_fitness = 125.67
Generation 2: avg_fitness = 128.90
...
Generation 31: avg_fitness = 145.23
```

#### 2. Анализ траекторий (Python)
```python
import pandas as pd
import matplotlib.pyplot as plt

# Загрузить данные
df = pd.read_csv('trajectories.csv')

# Визуализировать несколько траекторий
for traj_id in df['Trajectory'].unique()[:5]:
    traj = df[df['Trajectory'] == traj_id]
    plt.plot(traj['X'], traj['Y'], label=f'Traj {traj_id}')

plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Trajectories')
plt.grid(True)
plt.show()
```

#### 3. Проверка сохранённых параметров
```bash
cat best_params.txt
# Вывод: 41974.2 29423.1 53775.6 16406.0 41974.2 29423.1 53775.6 16406.0

wc -l best_matrix.txt
# 32 строк матрицы
```

### Продолжение оптимизации

Если нужно продолжить обучение с лучших результатов:

```cpp
// При следующем запуске с тем же main():
// 1. Файлы best_matrix.txt и best_params.txt будут обнаружены
// 2. Автоматически загрузятся в GA
// 3. Оптимизация продолжится с этой точки
// 4. Лучшие результаты снова перезапишут файлы
```

---

## Тесты

### Запуск

```bash
cd build
make nop_tests    # собрать тесты
./test/nop_tests  # запустить все тесты

# Запуск конкретного набора:
./test/nop_tests --gtest_filter="BaseFunctions.*"      # только базовые функции
./test/nop_tests --gtest_filter="GANOP_*"              # только GA-движок
./test/nop_tests --gtest_filter="NOP_FileIO.*"         # только файловый ввод/вывод
./test/nop_tests --gtest_filter="NOPminPsi.*"          # только мини-матрицы calcResult

# Или через ctest:
ctest
```

### Что проверяют тесты

| Набор | Что тестирует | Кол-во |
|-------|--------------|--------|
| `BaseFunctions` | Все 28 унарных (ro_1–ro_28) и 8 бинарных (xi_1–xi_8) функций: корректность, граничные значения, NaN/Inf защита | 37 |
| `NOP` / `NOPminPsi` | `calcResult()` с мини-матрицами (2×2, 3×3, 4×4) для каждой унарной и бинарной операции, плюс эталонный тест с 24×24 матрицей | 9 |
| `NOP_FileIO` | Сохранение/загрузка матриц и параметров, обработка несуществующих файлов, пустых данных, CSV с запятыми | 8 |
| `NOP_Genetic` | `GenVar()` — корректность вариаций (диагональ/недиагональ/добавление/удаление дуг), детерминированность с фиксированным seed | 3 |
| `NOP_EdgeCases` | Граничные случаи `calcResult()`: пустой ввод, большая матрица | 2 |
| `NOP_Output` | `printMatrix()`, `get_z()`, `get_parameters()` | 3 |
| `NOP_Reader` | XML-reader (`NOPMatrixReader`) | 1 |
| `NOP_Boundary` | Операции с Infinity, 0, отрицательными значениями | 3 |
| `GANOP_Grey` | Кодирование/декодирование Грея: roundtrip положительных, отрицательных и нулевых параметров | 4 |
| `NetOper_GenVar` | `GenVar()` с RNG: валидность вариаций, детерминированность | 2 |
| `NetOper_Variations` | `Variations()`: замена диагонали/недиагонали, добавление/удаление дуг, пустой вектор | 6 |
| `GANOP_Pareto` | Конструкция, `run()`, поиск Pareto-оптимальных решений | 3 |
| `GANOP_Crossover` | Кроссовер не крашится | 1 |
| `GANOP_Mutation` | Мутация сохраняет размер хромосомы | 1 |
| `GANOP_Integration` | Фитнес улучшается, воспроизводимость с seed, вызов колбэков | 3 |
| `GANOP_Edge` | Малая популяция, 100% мутация, 0 поколений, 1 параметр | 4 |
| `ModelControl` / `ModelState` / `ModelKinematics` | Арифметика `State`/`Control`, расстояния, кинематические формулы | 17 |
| `Controller` | Конструктор, расчёт управления | 2 |
| `Runner` | Полный цикл симуляции (требует ONNX модель) | 1 |

### Известные предсуществующие провалы

5 тестов падали **до** наших правок и связаны с устаревшими эталонными значениями или отсутствием ONNX модели:

- `NOP.simpleTestWithFunction` — эталонные значения для другой матрицы (14×14 вместо 24×24)
- `NOP.trainedOperatorTest` — `NopPsiN` не совпадает с XML-тестовыми данными
- `NOP.readMatrixAndParamsTests` — XML-файл содержит матрицу, отличающуюся от `NopPsiN`
- `Controller.SimpleTest` — `Umax=1.0` ограничивает выход, эталон не учитывает clamping
- `Runner.FullTest` — зависит от ONNX модели по относительному пути, golden values не совпадают

---

## Типичные ошибки и решения

### Ошибка 1: "File not found: rosbot_gazebo9_2d_model.onnx"
**Решение:**
```cpp
robot_config.model_path = "/полный/путь/к/модели/rosbot_gazebo9_2d_model.onnx";
```

### Ошибка 2: "Matrix is empty, nothing to save"
**Решение:** Убедись что `setPsi()` был вызван перед `saveMatrixToFile()`

---