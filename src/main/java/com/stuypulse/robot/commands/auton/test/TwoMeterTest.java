// package com.stuypulse.robot.commands.auton.test;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.swerve.SwerveResetPose;
// import com.stuypulse.robot.subsystems.swerve.Swerve;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class TwoMeterTest extends SequentialCommandGroup {

//     public TwoMeterTest(PathPlannerPath... paths) {
//         addCommands(new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
//                 Swerve.getInstance().followPathCommand(paths[0]));
//     }
// }
