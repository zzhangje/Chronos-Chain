# Soul Pivot: A Command-Based Robot Framework

<img width=100% src="./public/milk.jpg">

The architecture ensures that each subsystem interacts exclusively with interfaces, remaining completely unaware of whether the underlying implementation is a simulation or real hardware. Each subsystem exposes a minimal, consistent API. From these interfaces, atomic commands are designed to encapsulate discrete actions, enabling composition into sequences, parallel executions, or conditional workflows. This modular command structure allows for the fluid orchestration of complex robot behaviors, constructing the entire system as a hierarchy of reusable, testable, and interchangeable components.

![](./public/reefscape.svg) ![](./public/java.svg) ![](./public/akit.svg) ![](./public/ascope.svg)