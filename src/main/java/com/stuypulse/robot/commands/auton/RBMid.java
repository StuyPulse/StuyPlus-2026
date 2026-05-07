package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeCommands;
// import com.stuypulse.robot.commands.intake.IntakeAgitateWhileOuttaking;
// import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RBMid extends SequentialCommandGroup{
    public RBMid(PathPlannerPath...paths){
        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                /* .alongWith(new IntakeSetIntake())*/,

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),

            // new WaitCommand(2).deadlineFor(new IntakeAgitateWhileOuttaking().repeatedly()),

            IntakeCommands.setHomingDown()
        );
    }
    
}
