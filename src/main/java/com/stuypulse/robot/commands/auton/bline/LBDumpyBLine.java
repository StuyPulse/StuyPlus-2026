package com.stuypulse.robot.commands.auton.bline;

import java.util.function.Consumer;

import com.stuypulse.robot.commands.compound.TunableWaitCommand;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.BlineUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class LBDumpyBLine extends SequentialCommandGroup {

    private static final double HANDOFF_THRESHOLD_METERS = 0.15;

    public LBDumpyBLine(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(

            new TunableWaitCommand("LB Dumpy Delay"),
            new IntakeSetIntake(),
            swerve.getPathBuilder()
                .withPoseReset(swerve::resetPose)
                .build(new Path(BlineUtil.PATHS_DIR, pathNames[0]))
            
        );
    }

    private static Command followUntil(CommandSwerveDrivetrain swerve, String pathName, double thresholdMeters,
            Consumer<Pose2d> poseReset) {
        FollowPath path = (FollowPath) swerve.getPathBuilder().withPoseReset(poseReset)
                .build(new Path(BlineUtil.PATHS_DIR, pathName));

        return path.raceWith(
                new WaitUntilCommand(() -> path.getRemainingPathDistanceMeters() < thresholdMeters));

    }
}
