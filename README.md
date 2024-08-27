# ARGoS Hydroflock Implementation

**Status:** Work in Progress  
**Platform:** Ubuntu (tested)

The Argos Hydroflock Implementation is a robotics simulation project designed to test and develop advanced flocking behaviors in a swarm robotics environment using the ARGoS simulator. This project is actively under development and is currently tested on Ubuntu.

### Objective:

The Hydroflock project seeks to develop an intelligent, hydrodynamics-based flocking algorithm for swarm robotics, drawing inspiration from fluid dynamics principles. The objective is to create a cohesive and adaptive flocking behavior that mimics the way fluids flow and interact with obstacles. While similar implementations exist in known environments where flocking can be modeled at a high level, this project aims to extend these concepts into unknown environments, addressing the challenges of unpredictable and dynamic settings.

### Current Approach:

**Lennard-Jones Potential:** Currently, the project uses the Lennard-Jones potential model to govern interactions between robots, simulating attractive and repulsive forces that maintain group cohesion and avoid collisions.

### Challenges:

**Accurate Wall and Corner Detection:** The main challenge right now is to accurately detect and navigate around walls and corners in the simulation environment. This requires refining sensor data processing to reduce noise and jitter, effectively segmenting different sides of walls, and ensuring the robots can maintain cohesive flocking behavior while responding correctly to obstacles. Implementing these improvements in a computationally efficient manner is also crucial. When an outer corner is detected, the robots will aggregate towards the robot that found it, if this is not accurate then robots waste time aggregating somewhere that isn't a corner.<br><br>
**Direct Communication:** In ARGoS, direct communication is primarily implemented through the Range and Bearing sensor and actuator. Initially, time was invested in designing a routing protocol for this system (see the `rab-dsr` branch). However, due to its inherent broadcast-only nature, the system couldn't fully capitalize on the protocol's capabilities. To advance the development of the flocking algorithm, we've transitioned to using the omnidirectional camera for communication, detecting colors that represent different states and behaviors. These colors can be observed by neighboring robots, allowing individual flocking behaviors to adapt based on the states of nearby robots, ultimately achieving the desired emergent behavior at the swarm level.

### Future Directions:

**Fluid-Like Flocking Model:** The project intends to evolve towards a more sophisticated model that emulates the fluid-like behavior of robots, enabling smoother navigation around obstacles and better adaptation to dynamic environments.

## Table of Contents

- [Dependencies](#dependencies)
- [Installation](#installation)
- [Build and Run](#build-and-run)
- [Customization](#customization)
- [Logs and Debugging](#logs-and-debugging)
- [Development Notes](#development-notes)
- [Contributing](#contributing)
- [License](#license)

## Dependencies

This project relies on several external libraries and tools. Ensure you have the following dependencies installed on your system:

### Required Dependencies

1. **ARGoS** - [Website](https://www.argos-sim.info/)  
   ARGoS is the primary simulator for the Hydroflock project. It handles the simulation environment for the swarm robots.

2. **MLPack** - [GitHub](https://github.com/mlpack/mlpack)  
   Used for its DBSCAN implementation for clustering algorithms. <br>

   > Note: There is a naming conflict between ARGoS and MLPack. (see [below]{#})

3. **GNU Scientific Library (GSL)** - [Website](https://www.gnu.org/software/gsl/)  
   Utilized for linear regression and other numerical methods.

4. **Cereal** - [GitHub](https://github.com/USCiLab/cereal)  
   A header-only C++11 serialization library used for saving and loading simulation data.

5. **Armadillo** - [Website](http://arma.sourceforge.net/)  
   A C++ library for linear algebra & scientific computing, required by MLPack.

6. **Ensmallen** - [GitHub](https://github.com/mlpack/ensmallen)  
   A flexible C++ library for mathematical optimization, required by MLPack.

### Installing Dependencies

To install the required dependencies, run the following commands:

```bash
sudo apt-get update
sudo apt-get install libensmallen-dev libarmadillo-dev libopenblas-dev liblapack-dev libarpack2-dev libsuperlu-dev libmlpack-dev libgsl-dev
```

### Manual Installation of Cereal

`cereal` is not available as a prebuilt package. To install it manually:

1. Clone the repository:

   ```bash
   git clone https://github.com/USCiLab/cereal
   ```

2. Copy the header files to your system include directory:
   ```bash
   sudo cp -r ./cereal/include/cereal /usr/include
   ```

> **Note:** These libraries and their versions are subject to change as the project evolves.

## Installation

To build and run the project, follow these steps:

1. **Clone the Repository:**

   ```bash
   git clone https://your-repository-url.git
   cd your-repository-name
   ```

2. **Build the Project:**
   Use the provided shell script to build the project:

   ```bash
   ./build.sh
   ```

3. **Run the Simulation:**
   Use the following command to start the simulation:
   ```bash
   ./run.sh
   ```

## Customization

### Configuration

For advanced customization, you can modify the `.argos` file directly:

- **File Location:** `./experiments/hydroflock_dev.argos`
- **Python Script:** The `config.py` script is under development to simplify the configuration process but may not yet be fully functional.

## Logs and Debugging

### Log File Management

The project stores detailed logs in `./controllers/footbot_hydroflock/controller_logs/`. Each run creates a directory with a timestamp, storing individual log files for each robot. These logs are excluded from version control due to space considerations.

### Debugging Tools

- **`flog.sh`:** This script can filter and parse logs for easier debugging. Note that it is currently not actively maintained.
- **VSCode Configuration:** The project includes configurations for debugging with GDB in VSCode (`launch.json`, `tasks.json`, `c_cpp_properties.json`). These may require adjustments based on your environment.

## Development Notes

- **Platform:** This project is primarily developed and tested on Ubuntu.
- **IDE:** Visual Studio Code is recommended for its robust debugging and development tools.
  - **VSCode Extensions:** These are used in the codebase so if some of the comments are formatted weird it's probably because you don't have the extension. (_These are NOT required!_)
    - **Prettier** - For automatic formatting of code (makes it look "pretty"). It enforces a consistent style.
    - **:emojisense:** - Use emojis in your code comments! 😄
    - **Better Comments:** - Colorcoded and styled comments. Categorize your annotations and make comments a bit more readable.

#### "Log" Naming Conflict

- **Issue:**
  When integrating MLPack with ARGoS, a naming conflict arose because both libraries use `Log` for different purposes. ARGoS defines a `Log` macro for its math functions using logarithms, while MLPack has its own `Log` class. This conflict led to compilation errors.
- **Workaround:**
  Undefine `Log` before including the MLPack headers so that MLPack's `Log` class is recognized:
  ```cpp
  #undef Log
  #include <mlpack/core.hpp>
  #include <mlpack/methods/dbscan/dbscan.hpp>
  ```
  > **Note:**
  > If the `#undef Log` directive is applied globally or incorrectly, it could result in compilation errors in other parts of the ARGoS codebase that rely on the Log macro. As an alternative, if the conflict occurs frequently, consider wrapping MLPack-related code within a specific scope or using namespaces to reduce the likelihood of such conflicts.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

> **BEWARE:** The code does not follow any standard format or naming convention!

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.
