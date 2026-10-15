// package com.stuypulse.robot.commands.auton.defensive;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.intake.IntakeSetHomingDown;
// import com.stuypulse.robot.commands.swerve.SwerveResetPose;
// import com.stuypulse.robot.subsystems.swerve.Swerve;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class RBFerry extends SequentialCommandGroup {

//     public RBFerry(PathPlannerPath... paths) {
//         addCommands(
//                 new SwerveResetPose(paths[0].getStartingHolonomicPose().get()), Swerve.getInstance()
//                         .followPathCommand(paths[0]),
//                 Swerve.getInstance().followPathCommand(paths[1]),
//                 Swerve.getInstance().followPathCommand(paths[2]).alongWith(new IntakeSetHomingDown()),
//                 Swerve.getInstance().followPathCommand(paths[3]),
//                 new IntakeSetHomingDown());
//     }
// }
