package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToHub;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DoubleBump extends SequentialCommandGroup {
    
    public DoubleBump(PathPlannerPath ... paths){
        addCommands(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                .alongWith(new IntakeSetIntake()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new SwerveDriveAlignedToHub(),
                new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new FeederForward(), new IntakeAgitateOnce().repeatedly()),

                new FeederIdle().alongWith(new IntakeSetIntake())
        );

    }
}