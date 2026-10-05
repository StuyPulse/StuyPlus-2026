/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.shooter;

import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.shooter.ShooterConstants.ShooterSettings.FerryRPMInterpolation;
import com.stuypulse.robot.subsystems.shooter.ShooterConstants.ShooterSettings.FerryTOFInterpolation;
import com.stuypulse.robot.subsystems.shooter.ShooterConstants.ShooterSettings.RPMInterpolation;
import com.stuypulse.robot.subsystems.shooter.ShooterConstants.ShooterSettings.TOFInterpolation;
import com.stuypulse.robot.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolationCalculator {

    public static InterpolatingDoubleTreeMap shootingDistanceRPMInterpolator;

    public static InterpolatingDoubleTreeMap shootingDistanceTOFInterpolator;

    public static InterpolatingDoubleTreeMap ferryingDistanceRPMInterpolator;

    public static InterpolatingDoubleTreeMap ferryingDistanceTOFInterpolator;

    public record InterpolatedInfo(double targetRPM, double flightTimeSeconds) {
    }

    static {
        shootingDistanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : RPMInterpolation.distanceRPMInterpolationValues) {
            shootingDistanceRPMInterpolator.put(pair[0], pair[1]);
        }
        shootingDistanceTOFInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : TOFInterpolation.distanceTOFInterpolationValues) {
            shootingDistanceTOFInterpolator.put(pair[0], pair[1]);
        }
        ferryingDistanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : FerryRPMInterpolation.ferryDistanceRPMInterpolation) {
            ferryingDistanceRPMInterpolator.put(pair[0], pair[1]);
        }
        ferryingDistanceTOFInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : FerryTOFInterpolation.FerryTOFInterpolationInterpolation) {
            ferryingDistanceTOFInterpolator.put(pair[0], pair[1]);
        }
    }

    public static InterpolatedInfo interpolateShotInfo() {
        Swerve swerve = Swerve.getInstance();
        return interpolateShotInfo(swerve.getPose(), Field.getHubPose());
    }

    public static InterpolatedInfo interpolateShotInfo(Pose2d shooterPose, Pose2d targetPose) {
        Translation2d hubPose = targetPose.getTranslation();
        Translation2d currentPose = shooterPose.getTranslation();
        double distanceMeters = currentPose.getDistance(hubPose);
        double targetRPM = shootingDistanceRPMInterpolator.get(distanceMeters);
        double flightTime = shootingDistanceTOFInterpolator.get(distanceMeters);
        Logger.recordOutput("InterpolationTesting/Interpolated RPM", targetRPM);
        Logger.recordOutput("InterpolationTesting/Interpolated TOF", flightTime);
        // if (targetRPM < Settings.Shooter.MIN_SHOOTER_VELOCITY.in(RPM))
        //     targetRPM = Settings.Shooter.MIN_SHOOTER_VELOCITY.in(RPM);
        return new InterpolatedInfo(targetRPM, flightTime);
    }

    public static InterpolatedInfo interpolateFerryingInfo() {
        Swerve swerve = Swerve.getInstance();
        Pose2d shooterPose = swerve.getPose();
        Pose2d ferryPose = Field.getFerryZonePose(swerve.getPose().getTranslation());
        return interpolateFerryingInfo(shooterPose, ferryPose);
    }

    public static InterpolatedInfo interpolateFerryingInfo(Pose2d shooterPose, Pose2d targetPose) {
        Translation2d currentPose = shooterPose.getTranslation();
        Translation2d ferryPose = targetPose.getTranslation();
        double distanceMeters = currentPose.getDistance(ferryPose);
        double targetRPM = ferryingDistanceRPMInterpolator.get(distanceMeters);
        double flightTime = ferryingDistanceTOFInterpolator.get(distanceMeters);
        Logger.recordOutput("Shooter/Interpolated Ferry RPM", targetRPM);
        Logger.recordOutput("Shooter/Interpolated Ferry TOF", flightTime);
        return new InterpolatedInfo(targetRPM, flightTime);
    }
}
