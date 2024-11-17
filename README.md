# PIBT Multi-Agent Pathfinding

## Overview

Welcome to the **PIBT Multi-Agent Pathfinding** project! This repository implements the **Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding (PIBT)** algorithm. PIBT efficiently finds collision-free paths for multiple agents navigating a grid by prioritizing agents based on their planning difficulty. It’s designed for scalability and performance, making it ideal for real-time applications, such as autonomous vehicles, robotics, and games.

The project is set up using **CMake** for modularity, with tests written using **Google Test** to validate both the graph logic and the PIBT algorithm.

## Performance Stats:
- **High Density, Low Overhead**: PIBT handles **high-density environments** (up to 99% occupancy) with **low computational cost**. The average runtime for such environments is **0.0057 seconds**, making it suitable for real-time applications.
- **Optimal for Low Density**: In less-constrained environments (with fewer obstacles), PIBT can provide **near-optimal solutions**.
- **Suboptimal for High Density**: In highly congested grids, PIBT may offer **suboptimal solutions**, but the performance is still excellent and meets the needs of real-time systems where speed is a priority.

---

## Features

- **Priority-based Pathfinding**: Allocates priorities to agents based on their difficulty to ensure efficient planning.
- **Conflict Resolution**: Uses backtracking to resolve conflicts between agents, maintaining optimal paths.
- **Scalable Performance**: Handles high-density environments (up to 99% occupancy) while keeping computational overhead low.
- **Optimal and Suboptimal Solutions**: Provides near-optimal solutions in low-density grids and suboptimal solutions in high-density grids for real-time performance.
- **CMake-based Setup**: Hierarchical structure with modular components and easy management of dependencies.
- **Google Test Integration**: Unit tests for graph and PIBT functionality to ensure correctness.

---

## Getting Started

### Prerequisites

Ensure you have the following installed:

- **CMake** (v3.10 or higher)
- **Google Test** (for running tests)
- **C++ compiler** (e.g., GCC, Clang, MSVC)

### Installing Dependencies

1. Clone the repository:

    ```bash
    git clone git@github.com:arbazcodes/PIBT.git
    cd PIBT
    ```

2. Build the project using CMake:

    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```

### Running PIBT

To run the algorithm:

  ```bash
  ./main
  ```

### Running Tests

- To run all tests, navigate to the `build/` directory and execute the following command:
  
  ```bash
  ctest
  ```
- To run a specific test, use the -R option followed by the test name:
  ```bash
  ctest -R <Test_Name>
  ```
- To see detailed output for a specific test (including on failure), use the --output-on-failure flag:
-  ```bash
    ctest -R <Test_Name> --output-on-failure

- To build the project with tests enabled, configure the project with the following command:
  ```bash
  cmake .. -DENABLE_TESTS=ON



