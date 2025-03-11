# Adaptive Constrained Reinforcement Learning for Tracking Control of Unmanned Litter Collection Catamarans

This repository contains the official implementation of the paper "Adaptive Constrained Reinforcement Learning for Tracking Control of Unmanned Litter Collection Catamarans".

## Overview

This work proposes a novel adaptive constrained reinforcement learning approach for tracking control of unmanned litter collection catamarans. The algorithm combines adaptive control with constrained reinforcement learning to achieve robust tracking performance while ensuring safety constraints.

Key features:
- Adaptive learning mechanism for handling environmental uncertainties
- Safety-constrained optimization framework
- Real-time tracking control implementation
- Comprehensive simulation environment

## Requirements

- matlab 2020a (with MATLAB MinGW-w64 C/C++ Compiler)
- Webots 2022a
- Python (for helper scripts)

## Environment Setup

### Webots and MATLAB Configuration

1.  Install the MATLAB MinGW-w64 C/C++ Compiler

2.  Configure system environment variables:

    -   Set `WEBOTS_HOME` to your Webots installation path (e.g., `C:/Program Files/Webots`)

    -   Add to system `PATH`:
        -   %WEBOTS_HOME%/lib/controller
        -   %WEBOTS_HOME%/msys64/mingw64/bin
        -   MATLAB paths: `C:/Program Files/MATLAB/R2022b/bin` and `C:/Program Files/MATLAB/R2022b/bin/win64`

3.  Verify the setup by opening a sample world in Webots and checking if MATLAB launches automatically

#### Simulink Integration

1.  Create initialization file `boatSimulinkInit.m`:

    ```
    setenv('WEBOTS_PROJECT', 'path/to/your/project');
    setenv('WEBOTS_CONTROLLER_NAME', 'boatCtrlMatlabDemo');
    setenv('WEBOTS_VERSION', '2023a');
    
    cd(getenv("WEBOTS_HOME"));
    cd('lib/controller/matlab');
    launcher;
    ```

2.  Create controller file `boatCtrlMatlabDemo.m`:

    ```
    timestep = wb_robot_get_basic_time_step();
    leftMotor = wb_robot_get_device('left_propeller');
    rightMotor = wb_robot_get_device('right_propeller');
    uiopen('./boatCtrlDemo.slx', 1);
    ```

3.  Configure Simulink model `boatCtrlDemo.slx` with fixed step solver (0.01) to match Webots timestep

## Quick Start

1.  Open Webots and load the catamaran model:
    -   Set supervisor to `TRUE`
    -   Set controller to `<extern>`
2.  Without starting the simulation in Webots, open MATLAB and run: `boatSimulinkInit`
3.  When the Simulink model appears, use the Simulink controls to run and control the simulation.

## Details

The implementation includes:
- Adaptive constrained reinforcement learning algorithm
- Catamaran dynamics simulation
- MATLAB/Simulink tracking control implementation

## Citation

If you find this code useful in your research, please consider citing:

```bibtex
@article{author2024adaptive,
  title={Adaptive Constrained Reinforcement Learning for Tracking Control of Unmanned Litter Collection Catamarans},
  author={Author, A. and Author, B.},
  journal={Journal Name},
  year={2024}
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
