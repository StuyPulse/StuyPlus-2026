package com.stuypulse.robot.commands.auton.bline;

import java.util.function.Consumer;

import com.stuypulse.robot.commands.feeder.FeederSetForward;
import com.stuypulse.robot.commands.handoff.HandoffSetForward;
import com.stuypulse.robot.commands.intake.IntakeAgitateFastOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.shooter.ShooterFirstShotIncrease;
import com.stuypulse.robot.commands.shooter.ShooterSetShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForSpinUp;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.BlineUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class RBDumpyBLine extends SequentialCommandGroup {

    private static final double HANDOFF_THRESHOLD_METERS = 0.15;

    public RBDumpyBLine(String... pathNames) {
        
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(
            new IntakeSetIntake(),

            followUntil(swerve, pathNames[0], HANDOFF_THRESHOLD_METERS, swerve::resetPose),
            followUntil(swerve, pathNames[1], HANDOFF_THRESHOLD_METERS, swerve::resetPose),
            followUntil(swerve, pathNames[2], HANDOFF_THRESHOLD_METERS, swerve::resetPose),
            followUntil(swerve, pathNames[3], HANDOFF_THRESHOLD_METERS, swerve::resetPose),

            new SwerveDriveXMode(),
            new ShooterWaitForSpinUp(),
            new ShooterSetShoot(),
            new HandoffSetForward(),
            new ParallelDeadlineGroup(
                new WaitCommand(7),
                new FeederSetForward(), 
                new IntakeAgitateFastOnce().repeatedly(),
                new ShooterFirstShotIncrease()
            )
        );
    }

    private static Command followUntil(CommandSwerveDrivetrain swerve, String pathName, double thresholdMeters, Consumer<Pose2d> poseReset) {
        FollowPath path = (FollowPath) swerve.getPathBuilder()
            .withPoseReset(poseReset)
            .build(new Path(BlineUtil.PATHS_DIR, pathName));

        return path.raceWith(
            new WaitUntilCommand(() -> path.getRemainingPathDistanceMeters() < thresholdMeters)
        );
    }
}
