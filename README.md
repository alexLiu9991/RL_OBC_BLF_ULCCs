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

- matlab 2020a
- Webots 2022a

## Quick Start

```python
from src.algorithms import ACLR
from src.environment import CatamaranEnv

# Initialize environment
env = CatamaranEnv()

# Create and train the agent
agent = ACLR(env)
agent.train()

# Run tracking control
agent.run_tracking()
```

## Details

The implementation includes:
- Adaptive constrained reinforcement learning algorithm
- Catamaran dynamics simulation
- Tracking control implementation
- Performance evaluation tools

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

## Contact

For any questions or issues, please open an issue or contact [your-email@example.com].
