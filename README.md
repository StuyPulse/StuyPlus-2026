<div align="center">
    <img src="/images/logos/StuyPlusLogo.png" width="200" /> <img src="/images/logos/StuyPulseLogo.png" width="180" />
    <h1>Ron 2026</h1>
    <p>FRC Team StuyPlus <b>516</b> -- Very First Rookie Robot</p>
</div>

---

## Based on the WCP 2026 Competitive Concept for Rebuilt
https://wcproducts.com/pages/wcp-competitive-concepts

[![WCP 2026 Rebuilt Competitive Concept Video](https://img.youtube.com/vi/wO9aJNpCE8Q/0.jpg)](https://www.youtube.com/watch?v=wO9aJNpCE8Q)

## Subsystems and Features
- [Drivetrain](#drivetrain)
- [Intake](#intake)
- [Feeder](#feeder)
- [Shooter](#shooter)
- [Vision](#vision)
- [MapleSim w/ AdvantageScope](#maplesim-w-advantagescope)

## Drivetrain
File: [`src/main/java/com/stuypulse/robot/subsystems/swerve`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/swerve)

To be filled

## Intake
File: [`src/main/java/com/stuypulse/robot/subsystems/intake`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/intake)

To be filled

## Feeder
File: [`src/main/java/com/stuypulse/robot/subsystems/feeder`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/feeder)

To be filled

## Shooter
File: [`src/main/java/com/stuypulse/robot/subsystems/shooter`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/shooter)

To be filled

## Vision
Files: 
- [`src/main/java/com/stuypulse/robot/subsystems/vision`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/vision), 
- [`src/main/java/com/stuypulse/robot/util/vision`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/util/vision)

To be filled

## MapleSim w/ AdvantageScope
Files:
- [`src/main/java/com/stuypulse/robot/util/simulation/Simulation.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/Simulation.java)
- [`src/main/java/com/stuypulse/robot/util/simulation/SimulationConstants.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/SimulationConstants.java)
- [`src/main/java/com/stuypulse/robot/util/simulation/MapleSimSwerveDrivetrain.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/MapleSimSwerveDrivetrain.java)

There is also some code in [`CommandSwerveDrivetrain.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/subsystems/swerve/CommandSwerveDrivetrain.java) for: 
- Creating the `MapleSimSwerveDrivetrain`, 
- The update loop
- `NetworkTables` for publshing `SwerveStates`, `ChassisSpeeds`, and Drivetrain Pose.

Guide to usage: [`ascope_assets/README.md`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/ascope_assets/README.md)

---
[MapleSim](https://shenzhen-robotics-alliance.github.io/maple-sim/) is a physics simulation tool created in Java for FRC, made by the [Shenzhen Robotics Alliance](https://github.com/shenzhen-robotics-alliance). 

It uses the [dyn4j physics engine](https://github.com/dyn4j/dyn4j), simulating your Drivetrain as closely as your real code does, the field, and collisions between parts of the world. 

It works with AdvantageScope simply for rendering the Poses for all elements in the sim.

We use this for:
- Overall testing our archetype before [ME](## "Mechanical Engineering Department") passes it to us
- Visualize and test our code logic
- Test how auton might look on the field
- Help members understand how the robot will be operated and functioned when the robot is unavailable
- Visualize logs with physics

<img src="/images/references/maplesim/1.png" alt="Image of the robot on the field" />
<img src="/images/references/maplesim/2.png" alt="Image of the field right after the robot collided with the fuel in the neutral zone" />

## License
This project is under the [MIT License](/LICENSE)<div align="center">
    <img src="/images/logos/StuyPlusLogo.png" width="200" /> <img src="/images/logos/StuyPulseLogo.png" width="180" />
    <h1>Ron 2026</h1>
    <p>FRC Team StuyPlus <b>516</b> -- Very First Rookie Robot</p>
</div>

---

## Based on the WCP 2026 Competitive Concept for Rebuilt
https://wcproducts.com/pages/wcp-competitive-concepts

[![WCP 2026 Rebuilt Competitive Concept Video](https://img.youtube.com/vi/wO9aJNpCE8Q/0.jpg)](https://www.youtube.com/watch?v=wO9aJNpCE8Q)

## Subsystems and Features
- [Drivetrain](#drivetrain)
- [Intake](#intake)
- [Feeder](#feeder)
- [Shooter](#shooter)
- [Vision](#vision)
- [MapleSim w/ AdvantageScope](#maplesim-w-advantagescope)
- [TalonFX Simulation](#talonfx-simulation)

## Drivetrain
File: [`src/main/java/com/stuypulse/robot/subsystems/swerve`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/swerve)

To be filled

## Intake
File: [`src/main/java/com/stuypulse/robot/subsystems/intake`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/intake)

To be filled

## Feeder
File: [`src/main/java/com/stuypulse/robot/subsystems/feeder`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/feeder)

To be filled

## Shooter
File: [`src/main/java/com/stuypulse/robot/subsystems/shooter`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/shooter)

To be filled

## Vision
Files: 
- [`src/main/java/com/stuypulse/robot/subsystems/vision`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/vision), 
- [`src/main/java/com/stuypulse/robot/util/vision`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/util/vision)

To be filled

## MapleSim w/ AdvantageScope
Files:
- [`src/main/java/com/stuypulse/robot/util/simulation/Simulation.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/Simulation.java)
- [`src/main/java/com/stuypulse/robot/util/simulation/SimulationConstants.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/SimulationConstants.java)
- [`src/main/java/com/stuypulse/robot/util/simulation/MapleSimSwerveDrivetrain.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/MapleSimSwerveDrivetrain.java)

There is also some code in [`CommandSwerveDrivetrain.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/subsystems/swerve/CommandSwerveDrivetrain.java) for: 
- Creating the `MapleSimSwerveDrivetrain`, 
- The update loop
- `NetworkTables` for publshing `SwerveStates`, `ChassisSpeeds`, and Drivetrain Pose.

Guide to usage: [`ascope_assets/README.md`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/ascope_assets/README.md)

---
[MapleSim](https://shenzhen-robotics-alliance.github.io/maple-sim/) is a physics simulation tool created in Java for FRC, made by the [Shenzhen Robotics Alliance](https://github.com/shenzhen-robotics-alliance). 

It uses the [dyn4j physics engine](https://github.com/dyn4j/dyn4j), simulating your Drivetrain as closely as your real code does, the field, and collisions between parts of the world. 

It works with AdvantageScope simply for rendering the Poses for all elements in the sim.

We use this for:
- Overall testing our archetype before [ME](## "Mechanical Engineering Department") passes it to us
- Visualize and test our code logic
- Test how auton might look on the field
- Help members understand how the robot will be operated and functioned when the robot is unavailable
- Visualize logs with physics

<img src="/images/references/maplesim/1.png" alt="Image of the robot on the field" />
<img src="/images/references/maplesim/2.png" alt="Image of the field right after the robot collided with the fuel in the neutral zone" />

## TalonFX Simulation
File: [`src/main/java/com/stuypulse/robot/util/simulation/TalonFXSimulation.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/TalonFXSimulation.java)

To be filled

## License
This project is under the [MIT License](/LICENSE)