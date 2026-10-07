// package com.stuypulse.robot.commands.auton.defensive;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.swerve.SwerveResetPose;
// import com.stuypulse.robot.subsystems.swerve.Swerve;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class LBDisrupt extends SequentialCommandGroup {

//     public LBDisrupt(PathPlannerPath... paths) {
//         addCommands(new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
//                 Swerve.getInstance().followPathCommand(paths[0]),
//                 Swerve.getInstance().followPathCommand(paths[1]),
//                 Swerve.getInstance().followPathCommand(paths[2]),
//                 Swerve.getInstance().followPathCommand(paths[3]));
//     }
// }
