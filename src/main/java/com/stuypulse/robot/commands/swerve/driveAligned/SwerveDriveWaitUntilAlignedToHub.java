package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWaitUntilAlignedToHub extends Command {

    private final CommandSwerveDrivetrain instance;
    private final BStream isAligned;
    
    public SwerveDriveWaitUntilAlignedToHub() {
        this.instance = CommandSwerveDrivetrain.getInstance();
        this.isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));

        addRequirements(instance);
    }
    
    private Rotation2d getTargetAngle() {
        Transform2d diff = Field.getHubPose().minus(instance.getPose());
        return new Rotation2d(Math.atan(diff.getY() / diff.getX()));
    }

    private boolean isAligned() {
        Pose2d pose = instance.getPose();
        return Math.abs(pose.getRotation().minus(getTargetAngle()).getDegrees()) < Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getDegrees();
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }
}