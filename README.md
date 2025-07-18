# Chronos Chain

*A command-driven framework where robotic actions unfold in flawless sequence, like links in a timeless chain.*

<img width=100% src="./public/screenshot.png">

The architecture enforces strict interaction between subsystems solely through well-defined interfaces, ensuring complete abstraction from the underlying implementation—be it simulation or physical hardware. Each subsystem provides a minimal yet consistent API, from which atomic commands are derived to encapsulate discrete actions. These commands can then be composed into sequential operations, parallel executions, or conditional workflows, enabling flexible behavior orchestration. This modular command-based design facilitates the construction of the entire system as a hierarchical assembly of reusable, testable, and interchangeable components.

[![Watch on Bilibili](https://img.shields.io/badge/Watch%20on-Bilibili-ff69b4?logo=bilibili)](https://www.bilibili.com/video/BV1Pb3RzGEvL/) [![Project Page](https://img.shields.io/badge/Page-GitHub-blue?logo=github)](https://zhangzrjerry.github.io/projects/chronos-chain) ![](./public/java.svg) ![](./public/akit.svg) ![](./public/ascope.svg)

## Quick Start

See [Environment Setup](./docs/setup/)


## License

This code is licensed under the MIT License. It depends on WPILib and Advantage Kit, which are licensed under the BSD License.
