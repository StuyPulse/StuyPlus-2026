package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeSetHomingDown;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.commands.vision.SetVisionDisabled;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RightBumpMidStraight extends SequentialCommandGroup{

    public RightBumpMidStraight(PathPlannerPath...paths){
        addCommands(
            new SetVisionDisabled(),
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
            new IntakeSetIntake(),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            new IntakeSetHomingDown()
        );
    }
}