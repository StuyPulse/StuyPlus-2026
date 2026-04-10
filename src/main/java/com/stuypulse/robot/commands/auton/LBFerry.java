package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.intake.IntakeAgitateWhileOuttaking;
import com.stuypulse.robot.commands.intake.IntakeSetHomingDown;
// import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LBFerry extends SequentialCommandGroup {
    public LBFerry(PathPlannerPath... paths) {
        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                    /*alongWith(new IntakeSetIntake())*/,

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),

            new WaitCommand(2)/* .deadlineFor(new IntakeAgitateWhileOuttaking().repeatedly())*/,

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2])
                    .alongWith(new IntakeSetHomingDown()),
                    
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])
        );
    }
}