# Vacuum Simulation - README

## Introduction

This project simulates the behavior of an autonomous vacuum cleaner in a house environment. The vacuum's movement is controlled by different algorithms that explore the house, clean dirt, and return to the docking station when needed. This README provides detailed explanations of the core components of the project, including the `Explorer` class, the algorithms (`Algorithm_212346076_207177197_A` and `Algorithm_212346076_207177197_B`), and additional classes such as `Vacuum`, `House`, and `Simulation`. It also describes how these components interact with each other to create a cohesive simulation environment.

## Overview of Components and Interactions

### General Flow of the Program

1. **House Initialization**: The `House` class is initialized using a configuration file. This file defines the layout of the house, including walls, dirt levels, and the docking station.

2. **Simulation Setup**: The `Simulation` class orchestrates the overall process. It loads multiple house configurations and initializes algorithms for each house. The `Simulation` class runs these algorithms in parallel, simulating the vacuum's behavior in different houses.

3. **Algorithm Execution**: Each algorithm controls the vacuum's movement and cleaning actions within the house. The `Vacuum` class represents the vacuum cleaner, and the `Explorer` class is used by the algorithm to map the house and plan paths.

4. **Result Collection**: After running each simulation, the `Simulation` class collects the results, including the number of steps taken, the amount of dirt left, and the final score. The results are then summarized and saved to a file.

### Interactions Between Components

- **Vacuum and House**: The `Vacuum` interacts with the `House` by moving around, checking for walls, cleaning dirt, and monitoring its battery level. The `House` provides the layout and dirt levels that the vacuum must clean.

- **Algorithm and Explorer**: Each algorithm uses an instance of the `Explorer` class to maintain an internal map of the house. The algorithm updates this map as it explores new areas, detects walls, and cleans dirt. The `Explorer` also helps the algorithm find the shortest paths and make decisions based on the current state of the house.

- **Simulation and Algorithms**: The `Simulation` class manages the execution of algorithms across multiple houses. It initializes the `Explorer` and `Vacuum` for each algorithm, runs the simulation, and collects the results.

## Explorer Class

### Overview

The `Explorer` class is central to the navigation and mapping functions of the vacuum cleaner. Each algorithm has its own `Explorer` instance, which it uses to keep track of explored areas, dirt levels, walls, and distances from the docking station. The `Explorer` helps the algorithm decide where to move next and whether it needs to return to the docking station.

### Key Features

- **Mapping and Exploration**: The `Explorer` keeps track of all known areas in the house, including walls, dirt levels, and distances from the docking station. It also maintains a list of unexplored areas that the algorithm should visit.

- **Dirt Management**: The `Explorer` tracks the amount of dirt at each position and updates it as the vacuum cleans. This allows the algorithm to focus on areas that still have dirt.

- **Pathfinding**: The class provides methods to find the shortest path from the vacuum's current position to any other point in the house. It uses a custom implementation of the A* algorithm (`getShortestPath_A`) and a breadth-first search (BFS) based approach (`getShortestPath`).

- **Wall and Dock Detection**: The `Explorer` can determine if a specific position is a wall or the docking station, which helps the algorithm avoid obstacles and plan its route back to the dock.

### Important Functions

- **Mapping and Exploration**:
    - `bool explored(const Position pos) const`: Checks if a position has been explored.
    - `void setDirtLevel(const Position pos, int dirtLevel)`: Sets the dirt level at a specific position.
    - `void updateDirtAndClean(const Position pos, int dirtLevel)`: Updates the dirt level at a position and performs cleaning.

- **Pathfinding**:
    - `std::stack<Direction> getShortestPath_A(std::pair<int, int> src, std::pair<int, int> dst, bool search)`: Finds the shortest path using A*.
    - `std::stack<Direction> getShortestPath(std::pair<int, int> src, std::pair<int, int> dst, bool search)`: Finds the shortest path using BFS.
    - `Position getClosestUnexploredArea(Position position)`: Returns the closest unexplored area to a given position.

- **Wall and Dock Detection**:
    - `bool isWall(const Position pos)`: Checks if a position is a wall.
    - `bool isDockingStation(const Position pos)`: Checks if a position is the docking station.

## Algorithms

### Algorithm_212346076_207177197_A

#### Overview

`Algorithm_212346076_207177197_A` is a depth-first search (DFS) based algorithm that controls the vacuum's movement. It explores the house by prioritizing unexplored areas and returns to the dock when necessary. This algorithm is simple but effective for exploring large areas.

#### Key Features

- **Exploration**: The algorithm prioritizes exploring new areas. It uses DFS to move to adjacent unexplored areas while avoiding walls. The `Explorer` is updated as new areas are discovered.

- **Cleaning**: When dirt is detected, the algorithm switches to cleaning mode and stays in place until the dirt is cleaned. The `Explorer` is updated with the new dirt levels.

- **Docking**: The algorithm monitors the battery level and returns to the dock when it's low, using the shortest path provided by the `Explorer`.

#### Important Functions

- **State Management**:
    - `void setMaxSteps(std::size_t maxSteps)`: Sets the maximum number of steps the algorithm can take.
    - `bool StateChanged() const`: Checks if the state has changed.
    - `State getCurrentState() const`: Returns the current state of the algorithm.

- **Exploration and Cleaning**:
    - `Step nextStep() override`: Determines the next step for the vacuum to take.
    - `void updateExplorerInfo(Position current_position_)`: Updates the `Explorer` with information about the current position, such as dirt levels and walls.

- **Pathfinding**:
    - `int getMinDistanceOfNeighbors(const Position& curr_pos)`: Gets the minimum distance to the docking station from the neighboring positions.
    - `void updatePosition(Step stepDirection, Position& current_pos)`: Updates the position of the vacuum based on the step direction.

#### States

- **EXPLORE**: The default state, where the algorithm looks for unexplored areas using DFS.
- **CLEANING**: Activated when dirt is detected; the vacuum stays in place until the dirt is cleaned.
- **TO_DOCK**: Activated when the battery is low; the vacuum returns to the docking station using the shortest path.
- **CHARGING**: The vacuum stays at the docking station until fully charged.
- **TO_POS**: Used to move to a specific position, such as the closest unexplored area.
- **FINISH**: The final state, activated when all areas are explored or the maximum steps are reached.

### Algorithm_212346076_207177197_B

#### Overview

`Algorithm_212346076_207177197_B` is an enhanced algorithm that combines BFS for exploration with optimized pathfinding to navigate the house. It ensures efficient exploration and cleaning while considering battery constraints.

#### Key Features

- **BFS Queue**: The algorithm uses a BFS queue to prioritize exploration of nearby unexplored areas. The `Explorer` helps maintain the queue and updates it as new areas are discovered.

- **State Management**: The algorithm uses different states (similar to `Algorithm_212346076_207177197_A`) to handle exploration, cleaning, and docking. The `Explorer` provides the necessary information to make state transitions.

- **Battery Optimization**: The algorithm continuously monitors the battery level and calculates the shortest path back to the docking station, ensuring that the vacuum never runs out of power far from the dock.

#### Important Functions

- **State Management**:
    - `void setMaxSteps(std::size_t maxSteps)`: Sets the maximum number of steps the algorithm can take.
    - `bool StateChanged() const`: Checks if the state has changed.
    - `State getCurrentState() const`: Returns the current state of the algorithm.

- **Exploration and Cleaning**:
    - `Step nextStep() override`: Determines the next step for the vacuum to take.
    - `void updateExplorerInfo(Position current_position_)`: Updates the `Explorer` with information about the current position, such as dirt levels and walls.

- **Pathfinding**:
    - `int getMinDistanceOfNeighbors(const Position& curr_pos)`: Gets the minimum distance to the docking station from the neighboring positions.
    - `void updatePosition(Step stepDirection, Position& current_pos)`: Updates the position of the vacuum based on the step direction.

#### States

- **EXPLORE**: The algorithm explores the house using BFS, adding nearby unexplored areas to a queue for later exploration.
- **CLEANING**: Activated when dirt is detected; the vacuum stays in place until the dirt is cleaned.
- **TO_DOCK**: The vacuum returns to the dock when the battery is low or if it is time to finish the task.
- **CHARGING**: The vacuum stays at the docking station until fully charged.
- **TO_POS**: Moves the vacuum to the next position in the BFS queue or the closest unexplored area.
- **FINISH**: The final state, activated when all areas are explored or the maximum steps are reached.

## Vacuum Class

### Overview

The `Vacuum` class models the physical attributes of the vacuum cleaner, such as its battery level, position, and movement. It interacts closely with the algorithms and the house layout to perform cleaning tasks.

### Key Features

- **Battery Management**: The vacuum keeps track of its battery level, which decreases as it moves. The vacuum can recharge its battery when it is at the docking station.

- **Movement**: The vacuum can move in four directions (North, South, East, West) and can stay in place (`Step::Stay`). The movement consumes battery, except when staying at the docking station.

- **Position Tracking**: The vacuum tracks its current position within the house, which is crucial for both navigation and cleaning.

### Important Functions

- **Initialization and Battery Management**:
    - `void init(double battery, Position position)`: Initializes the vacuum with a given battery level and starting position.
    - `double maxBattery() const`: Returns the maximum battery capacity.
    - `double battery() const`: Returns the current battery level.
    - `void charge()`: Charges the vacuum's battery if it is at the docking station.

- **Movement and Position Tracking**:
    - `void step(Step stepDirection)`: Moves the vacuum in the specified direction, consuming battery.
    - `Position getPosition() const`: Returns the current position of the vacuum.
    - `bool atDockingStation() const`: Checks if the vacuum is at the docking station.

## House Class

### Overview

The `House` class represents the house environment in which the vacuum operates. It stores the layout of the house, including walls, dirt levels, and the position of the docking station. The house is initialized from a configuration file.

### Key Features

- **Layout Initialization**: The house layout is initialized from a configuration file that defines the dimensions of the house, the position of walls, dirt levels, and the docking station.

- **Dirt Management**: The house keeps track of the total amount of dirt and updates the dirt levels as the vacuum cleans.

- **Validation**: The house validates that the configuration file includes a docking station and adheres to the expected format.

### Important Functions

- **Layout and Dirt Management**:
    - `int getRows() const`: Returns the number of rows in the house layout.
    - `int getCols() const`: Returns the number of columns in the house layout.
    - `int getDirtLevel(const Position& pos) const`: Returns the dirt level at a specific position.
    - `void cleanCell(const Position& pos)`: Reduces the dirt level at a specific position.

- **Wall and Dock Detection**:
    - `bool isWall(const Position& pos) const`: Checks if a given position is a wall.
    - `bool isInDock(const Position& pos) const`: Checks if a given position is the docking station.

- **Utility**:
    - `int getTotalDirt() const`: Returns the total amount of dirt in the house.
    - `bool isHouseClean() const`: Returns true if all the dirt in the house has been cleaned.
    - `void printHouseMatrix() const`: Prints the house matrix layout to the console.

## Simulation Class

### Overview

The `Simulation` class orchestrates the execution of the algorithms on different house layouts. It manages multiple simulations in parallel and collects the results, including the number of steps taken, the amount of dirt left, and the final score.

### Key Features

- **Parallel Execution**: The simulation can run multiple algorithms on different house layouts concurrently using multiple threads.

- **Result Collection**: The simulation collects results for each algorithm and house combination, including the steps taken, dirt left, and whether the vacuum finished in the docking station.

- **Score Calculation**: The simulation calculates a score for each algorithm based on the number of steps taken, the amount of dirt left, and whether the vacuum finished in the docking station. Lower scores are better.

### Important Functions

- **Simulation Control**:
    - `void runSimulations(const std::vector<std::pair<std::string, std::function<std::unique_ptr<AbstractAlgorithm>()>>>& algorithms, int numThreads, bool summaryOnly)`: Runs the simulations for the given algorithms and house layouts.
    - `void runSingleSimulation(const House& house, std::unique_ptr<AbstractAlgorithm> algo, const std::string& algoName, int maxSteps, int maxBattery, bool summaryOnly)`: Runs a single simulation of an algorithm on a house layout.

- **Result Collection and Scoring**:
    - `SimulationResult simulateAlgorithm(House& house, AbstractAlgorithm& algo, int maxSteps, int maxBattery)`: Simulates the behavior of an algorithm on a given house layout.
    - `int calculateScore(const SimulationResult& result, int maxSteps, int initialDirt) const`: Calculates the score for a simulation based on the result.
    - `void generateSummary() const`: Generates a summary of the simulation results, saving them to a CSV file.

## ConfigReader Class

### Overview

The `ConfigReader` class reads and parses configuration files that define the house layouts used in the simulations. It extracts key information such as the house layout, maximum steps, and battery capacity.

### Key Features

- **File Parsing**: The class reads the configuration file line by line and extracts relevant information using regular expressions.

- **Validation**: The class ensures that the configuration file includes all necessary information and that there is exactly one docking station.

### Important Functions

- **Configuration Parsing**:
    - `ConfigReader(const std::string &file_path)`: Constructor that reads and parses a configuration file.
    - `const std::vector<std::string>& getLayout() const`: Returns the house layout as a vector of strings.
    - `int getMaxSteps() const`: Returns the maximum number of steps allowed in the simulation.
    - `int getMaxBattery() const`: Returns the maximum battery capacity.
    - `const std::string& getHouseName() const`: Returns the name of the house.

## How to Build and Run the Program

### Building the Project

To build the project, use the following commands:

```bash
cd Vacuum_Final
rm -rf build
mkdir build
cmake -S . -B ./build
cmake --build ./build
