package com.stuypulse.robot.commands.auton.shooting;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.handoff.HandoffSetForward;
import com.stuypulse.robot.commands.intake.IntakeAgitateFastOnce;
<<<<<<< Updated upstream
=======
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
>>>>>>> Stashed changes
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForSpinUp;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
<<<<<<< Updated upstream
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignToHub;
=======
>>>>>>> Stashed changes
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LBDumpy extends SequentialCommandGroup {
    public LBDumpy(PathPlannerPath... paths) {
        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
<<<<<<< Updated upstream
=======
            new IntakeSetIntake(),
>>>>>>> Stashed changes
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
<<<<<<< Updated upstream
            new SwerveDriveAlignToHub(),
=======
>>>>>>> Stashed changes
            new SwerveDriveXMode(),
            new ShooterWaitForSpinUp(),
            new ShooterSetShoot(),
            new HandoffSetForward(),
            new ParallelDeadlineGroup(
<<<<<<< Updated upstream
                new WaitCommand(6.5),
                new FeederSetForward(), 
                new IntakeAgitateFastOnce().repeatedly()
            )
        );
=======
                new WaitCommand(7),
                new FeederSetForward(), 
                new IntakeAgitateFastOnce().repeatedly()
        )
>>>>>>> Stashed changes
    }
}