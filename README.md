<div align="center">
    <img src="/images/logos/StuyPlusLogo.png" width="200" /> <img src="/images/logos/StuyPulseLogo.png" width="180" />
    <h1>Ron 2026</h1>
    <p>FRC Team StuyPlus <b>516</b> - First In-season Rookie Robot</p>
</div>

---

## Based on the [WCP 2026 Competitive Concept](https://wcproducts.com/pages/wcp-competitive-concepts) for Rebuilt

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

Our intake is made up of 3 sets of rollers, all connected by belts, which are stowed and deployed with a pivot attached to the roller plate.

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
How to use our MapleSim implementation on **your** computer: [`MapleSim Usage Guide`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/ascope_assets/README.md)

Files:
- [`Simulation.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/Simulation.java)
- [`SimulationConstants.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/SimulationConstants.java)
- [`MapleSimSwerveDrivetrain.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/simulation/MapleSimSwerveDrivetrain.java)

Additionally, we modified our [`CommandSwerveDrivetrain.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/subsystems/swerve/CommandSwerveDrivetrain.java) in order to: 
- Replace the normal simulation instance with a `MapleSimSwerveDrivetrain` instance
- Update the simulation instance with Maplesim's periodic method (`MapleSimSwerveDrivetrain::update`)
- Publish our simulated swerve module states, chassis speeds, and drivetrain pose to `NetworkTables`

---
[MapleSim](https://shenzhen-robotics-alliance.github.io/maple-sim/) is a physics simulation tool created in Java for FRC, made by the [Shenzhen Robotics Alliance](https://github.com/shenzhen-robotics-alliance). 

It uses the [dyn4j physics engine](https://github.com/dyn4j/dyn4j), simulating your Drivetrain as closely as your real code does, the field, and collisions between parts of the world. 

It works with AdvantageScope simply for rendering the Poses for all elements in the sim.

We use this for:
- Overall testing our archetype before our Mechanical Engineering department finishes the physical robot
- Visualizing and testing our code logic
- Testing how autons might look on the field
- Helping members understand how the robot will operate
- Visualize match logs in 3d

## License
This project is under the [MIT License](/LICENSE)
