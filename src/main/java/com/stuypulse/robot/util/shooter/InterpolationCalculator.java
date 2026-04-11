package com.stuypulse.robot.util.shooter;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.util.shooter.Interpolation;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InterpolationCalculator {
    public static InterpolatingDoubleTreeMap distanceRPMInterpolator;
    public static InterpolatingDoubleTreeMap distanceTOFShootingInterpolator;
    public static InterpolatingDoubleTreeMap distanceTOFFerryingInterpolator;
    public static InterpolatingDoubleTreeMap ferryingDistanceRPMInterpolator;

    static {
        distanceRPMInterpolator = Interpolation.Shooting.getInterpolator();
        ferryingDistanceRPMInterpolator = Interpolation.Ferrying.getInterpolator();
        distanceTOFShootingInterpolator = Interpolation.TOFShooting.getInterpolator();
        distanceTOFFerryingInterpolator = Interpolation.TOFFerrying.getInterpolator();
    }
    
    
    public record InterpolatedShotInfo(double targetRPM, double flightTimeSeconds) {
    }

    public static InterpolatedShotInfo interpolateShotInfo() {
        return interpolateShotInfo(Field.getHubPose());
    }

    public static InterpolatedShotInfo interpolateShotInfo(Pose2d targetPose) {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        Translation2d hubPose = targetPose.getTranslation();
        Translation2d shooterPose = swerve.getShooterPose().getTranslation();

        double distanceMeters = shooterPose.getDistance(hubPose);

        double targetRPM = distanceRPMInterpolator.get(distanceMeters);
        double flightTime = distanceTOFShootingInterpolator.get(distanceMeters);

        SmartDashboard.putNumber("HoodedShooter/Interpolated RPM", targetRPM);
        SmartDashboard.putNumber("HoodedShooter/Interpolated TOF", flightTime);

        return new InterpolatedShotInfo(
            targetRPM, 
            flightTime
        );
    }

    public static Supplier<InterpolatedShotInfo> interpolateFerryingRPM() {
        return () -> {
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

            Translation2d currentPose = swerve.getShooterPose().getTranslation();
            Translation2d cornerPose = Field.getFerryZonePose(currentPose).getTranslation();

            double distanceMeters = cornerPose.getDistance(currentPose);

            double targetRPM = ferryingDistanceRPMInterpolator.get(distanceMeters);
            double flightTime = distanceTOFFerryingInterpolator.get(distanceMeters);
            
            return new InterpolatedShotInfo(targetRPM, flightTime);
        };
    }   
}