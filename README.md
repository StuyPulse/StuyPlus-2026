<div align="center">
    <img src="/assets/logos/StuyPlusLogo.png" width="200" /> <img src="/assets/logos/StuyPulseLogo.png" width="180" />
    <h1>Ron 2026</h1>
    <p>FRC Team StuyPlus <b>516</b> - First regular season Rookie Robot</p>
</div>
<div align="center">

[![WPILib](https://img.shields.io/badge/WPILib-v2026.2.1-AB1A2D?style=for-the-badge&logo=first)](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.2.1)
[![Java Version](https://img.shields.io/badge/Java-17-F29111?style=for-the-badge&logo=openjdk)](https://jdk.java.net/17/)
[![Java Version](https://img.shields.io/badge/C++-20-659AD2?style=for-the-badge&logo=cplusplus)](../cpp)
[![License](https://img.shields.io/badge/License-MIT-750014?style=for-the-badge&logo=markdown)](LICENSE)
<br>
![Build](https://img.shields.io/github/actions/workflow/status/StuyPulse/StuyPlus-2026/gradle.yml?style=for-the-badge&label=Build)

</div>

---

## Table of Contents
- [Inspiration](#based-on-the-wcp-2026-competitive-concept-for-rebuilt)
- [Documentation](#documentation)
- [Branch Naming Convention](#branch-naming-convention)
- Subsystems
    - [Drivetrain](#drivetrain)
    - [Intake](#intake)
    - [Feeder](#feeder)
    - [Shooter](#shooter)
    - [Vision](#vision)
- [MapleSim w/ AdvantageScope](#maplesim-w-advantagescope)
- [Credits](#credits)
- [License](#license)

## Based on the [WCP 2026 Competitive Concept](https://wcproducts.com/pages/wcp-competitive-concepts) for Rebuilt

[![WCP 2026 Rebuilt Competitive Concept Video](https://www.video-thumbnail.com/youtube/wO9aJNpCE8Q)](https://www.youtube.com/watch?v=wO9aJNpCE8Q)

## Documentation
The documentation for StuyPlus-2026 can be found [here](https://stuypulse.github.io/StuyPlus-2026/).

## Branch Naming Convention
| Prefix | Use Case |
|--------|---------|
| `main` | Production branch |
| `feat/` | New features that may need some review before being pushed |
| `experiments/` | Experimental work, may or may not necessarily be pushed to `main` |
| `docs` | Not meant to be pushed to manually. Only for the [`javadoc.yml`](./.github/workflows/javadoc.yml) action for automatic [documentation page](https://stuypulse.github.io/StuyPlus-2026/) generation |
| `feat/doglog` | Not meant to be pushed to manually. Only for the [`doglog-replacement.yml`](./.github/workflows/doglog-replacement.yml) action. Intended to just keep the Doglog option open without necessarily pushing to main. |


## Subsystems and Features

## Drivetrain
Files: 
- [`src/main/java/com/stuypulse/robot/subsystems/swerve`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/swerve)
- [`src/main/java/com/stuypulse/robot/util/HolonomicController.java`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/util/HolonomicController.java)

We utilize a raised swerve drivetrain with CTRE Phoenix 6 hardware and four swerve modules, letting us independently rotate the wheels. This allows for higher manueverability.

For our [MapleSim w/ AdvantageScope](#maplesim-w-advantagescope), we create a MapleSimSwerveDrivetrain for simulation purposes and publish drivetrain pose, module states, and chassis speeds via `NetworkTables`.

The robot is driven `Field-Centric`.

We use both [Vision](#vision) and odometry for pose estimation. Check the vision section for details. 

`HolonomicController.java`, made by the main team 694, is essentially the PID Controller but for the drivetrain.

## Intake
File: [`src/main/java/com/stuypulse/robot/subsystems/intake`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/intake)

Our intake is made up of 3 sets of rollers, all connected by belts, which are stowed and deployed with a pivot attached to the roller plate.

It contains the following states:
- `IDLE`: Intake brought up, rollers do not run
- `INTAKE`: Intake brought down, rollers run forward on a duty cycle
- `OUTTAKE`: Intake brought down, rollers run backward on a duty cycle
- `DOWN`: Intake brought down, rollers do not run
- `HOMING_DOWN`: Intake pushed down, rollers do not run

Based on an angle and a duty cycle, in the `periodic` method, we use PID to control our pivot and a duty cycle to control the percentage of power given to the rollers.

In order to stop the fuel from pushing the intake up, we apply something called "pushdown current". This keeps pushing the intake downwards in order to resist the force of the fuel and keep it downwards at the angle we want.

Due to encoder issues when the chain skips, it's quite difficult to detect when the pivot is within tolerance. In order to detect this, we detect stalling to know when to stop. This works in combination with homing down, in which we apply a constant current in order to force the pivot downwards.

## Feeder
File: [`src/main/java/com/stuypulse/robot/subsystems/feeder`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/feeder)

Our feeder is of indexer type, having three lanes for  the [shooter](#shooter)'s three slots. It uses two motors (follower-leader) to guide the fuel from the hopper to the shooter.

It contains the following states:
- `STOP`: Feeder is stopped
- `FORWARD`: Motors run forward on a duty cycle to feed fuel to the shooter
- `REVERSE`: Motors run backward on a duty cycle to work with the intake to outtake fuel from the robot

In the `periodic` method, we use `DutyCycleOut` to control the percentage of power given to both feeder motors. The feeder is set to only run when aligned to the hub if in `SHOOT` state. For SOTM/FOTM, it still feeds while moving if needed.

## Shooter
File: [`src/main/java/com/stuypulse/robot/subsystems/shooter`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/shooter)

Our shooter is made up of 3 shooter motors (`Left`, `Center`, `Right`).

The `Center` and `Right` motors follow the `Left` motor.

> [!IMPORTANT]
> SOTM (Shoot on the move) and FOTM (Ferry on the move) are low priority.

It contains the following states:
- `IDLE`: Shooter doesn't run.
- `SHOOT`: Shooter wheels spin at it's target RPM, interpolated based on distance to hub.
- `SOTM`: Shoot on the move. Shooter RPM is interpolated based on distance to hub.
- `FOTM`: Ferry on the move. Shooter RPM is interpolated based on distance to hub.
- `FERRY`: Shooter wheels spin at it's target RPM, interpolated based on distance to ferry zone.

In the `periodic` method, the shooter RPM is controlled via `VelocityTorqueCurrentFOC` control request.

## Handoff
File: [`src/main/java/com/stuypulse/robot/subsystems/handoff`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/handoff)

The handoff is a singular motor that helps the feeder by "handing off" the fuel from the hopper to the feeder indexer lanes.

It contains the following states:
- `IDLE`: Handoff doesn't run.
- `FORWARD`: The handoff runs forward
- `REVERSE`: The handoff runs backward

In the `periodic` method, the handoff motors use DutyCycleOut to control the handoff.

## Vision
Files: 
- [`src/main/java/com/stuypulse/robot/subsystems/vision`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/subsystems/vision), 
- [`src/main/java/com/stuypulse/robot/util/vision`](https://github.com/StuyPulse/StuyPlus-2026/tree/main/src/main/java/com/stuypulse/robot/util/vision)

This uses Limelights from [Limelight Vision](https://limelightvision.io/) to use the AprilTags to estimate the pose of the robot. It works in combination with odometry to estimate the pose using gyros and encoders when using megatag 2.

All of the math and code is mostly done within the Limelight itself via LimelightOS. You mainly just need to connect it to your robot and determine the protocol  it sends to.

[`LimelightHelpers.java`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/src/main/java/com/stuypulse/robot/util/vision/LimelightHelpers.java) is a wrapper class for the vision `NetworkTables` from the Limelight that abstracts many functions for you such as setting pipelines and the pose estimation. You can find the latest version of `LimelightHelpers.java` [here](https://github.com/LimelightVision/limelightlib-wpijava/releases).

Features:
- The robot currently only has one Limelight
- Uses [MegaTag](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization) localization algorithm
- Commands for switching the Limelight pipeline based on `Sunny` and `Cloudy` conditions to increase efficiency.

## MapleSim w/ AdvantageScope
The usage guide can be found [here.](https://github.com/StuyPulse/StuyPlus-2026/blob/main/assets/ascope/README.md)

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
- Visualizing match logs in 3d

<img src="/assets/readme/maplesim/1.png" alt="Image of the robot a the starting position" />
<img src="/assets/readme/maplesim/2.png" alt="Image of the field right after the robot collided with the fuel in the neutral zone" />
<img src="/assets/readme/maplesim/3.png" alt="Image of the side of the robot with the intake out">

## Credits
The styling in [`betterjdocs.css`](https://github.com/StuyPulse/StuyPlus-2026/blob/main/assets/javadoc/betterjdocs.css) is based on a modified version of BetterJDocs.
<br>
Credit to [@xMrAfonso](https://github.com/xMrAfonso) and [@Andre601](https://github.com/Andre601) for the original file:
[https://github.com/xMrAfonso/BetterJDocs](https://github.com/xMrAfonso/BetterJDocs)

## License
This project is under the [MIT License](/LICENSE)
