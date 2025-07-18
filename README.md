# Chronos Chain

*A command-driven framework where robotic actions unfold in flawless sequence, like links in a timeless chain.*

<img width=100% src="./public/screenshot.png">

The architecture enforces strict interaction between subsystems solely through well-defined interfaces, ensuring complete abstraction from the underlying implementation—be it simulation or physical hardware. Each subsystem provides a minimal yet consistent API, from which atomic commands are derived to encapsulate discrete actions. These commands can then be composed into sequential operations, parallel executions, or conditional workflows, enabling flexible behavior orchestration. This modular command-based design facilitates the construction of the entire system as a hierarchical assembly of reusable, testable, and interchangeable components.

![](./public/reefscape.svg) ![](./public/java.svg) ![](./public/akit.svg) ![](./public/ascope.svg)
