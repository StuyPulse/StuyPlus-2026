package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeCommands;
// import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.commands.vision.SetVisionDisabled;
import com.stuypulse.robot.commands.vision.SetVisionEnabled;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LBMid extends SequentialCommandGroup{
    public LBMid(PathPlannerPath...paths){
        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                /* .alongWith(new IntakeSetIntake())*/,

            new SetVisionDisabled(),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),

            IntakeCommands.setHomingDown(),
            
            new SetVisionEnabled()
        );
    }
    
}
