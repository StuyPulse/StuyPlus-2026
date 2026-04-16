/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.shooter;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Shooter.FerryRPMInterpolation;
import com.stuypulse.robot.constants.Settings.Shooter.RPMInterpolation;
import com.stuypulse.robot.constants.Settings.Shooter.TOFInterpolation;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InterpolationCalculator {

    public static InterpolatingDoubleTreeMap distanceAngleInterpolator;
    public static InterpolatingDoubleTreeMap distanceRPMInterpolator;
    public static InterpolatingDoubleTreeMap distanceTOFInterpolator;

    public static InterpolatingDoubleTreeMap ferryingDistanceRPMInterpolator;

    public record InterpolatedShotInfo(
        // Rotation2d targetHoodAngle,
        double targetRPM,
        double flightTimeSeconds) {
    }

    public record InterpolatedFerryInfo(
        // Rotation2d targetHoodAngle,
        double targetRPM,
        double flightTimeSeconds) {   
    }
    
    static {
        distanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : RPMInterpolation.distanceRPMInterpolationValues) {
            distanceRPMInterpolator.put(pair[0], pair[1]);
        }

        distanceTOFInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : TOFInterpolation.distanceTOFInterpolationValues) {
            distanceTOFInterpolator.put(pair[0], pair[1]);
        }

        ferryingDistanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for(double[] pair: FerryRPMInterpolation.ferryDistanceRPMInterpolation) {
            ferryingDistanceRPMInterpolator.put(pair[0], pair[1]);
        }
    }
    
    public static InterpolatedShotInfo interpolateShotInfo(){
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        return interpolateShotInfo(swerve.getPose(), Field.getHubPose());
    }

    public static InterpolatedShotInfo interpolateShotInfo(Pose2d shooterPose, Pose2d targetPose) {
        Translation2d hubPose = targetPose.getTranslation();
        Translation2d currentPose = shooterPose.getTranslation();

        double distanceMeters = currentPose.getDistance(hubPose);

        double targetRPM = distanceRPMInterpolator.get(distanceMeters);
        double flightTime = distanceTOFInterpolator.get(distanceMeters);
        

        SmartDashboard.putNumber("InterpolationTesting/Interpolated RPM", targetRPM);
        SmartDashboard.putNumber("InterpolationTesting/Interpolated TOF", flightTime);

        return new InterpolatedShotInfo(
            targetRPM, 
            flightTime
        );
    }
    
    public static InterpolatedFerryInfo interpolateFerryingInfo() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        Pose2d shooterPose = swerve.getPose();
        Pose2d ferryPose = Field.getFerryZonePose(swerve.getPose().getTranslation());

        double distanceMeters = shooterPose.getTranslation().getDistance(ferryPose.getTranslation());

        double targetRPM = distanceRPMInterpolator.get(distanceMeters);
        double flightTime = distanceTOFInterpolator.get(distanceMeters);

        SmartDashboard.putNumber("InterpolationTesting/Ferry Interpolated RPM", targetRPM);
        SmartDashboard.putNumber("InterpolationTesting/Ferry Interpolated TOF", flightTime);
        
        return interpolateFerryingInfo(
            shooterPose,
            Field.getFerryZonePose(shooterPose.getTranslation())
        );
    }

    public static InterpolatedFerryInfo interpolateFerryingInfo(Pose2d shooterPose, Pose2d targetPose) {
        Translation2d currentPose = shooterPose.getTranslation();
        Translation2d ferryPose = targetPose.getTranslation();

        double distanceMeters = currentPose.getDistance(ferryPose);

        double targetRPM = ferryingDistanceRPMInterpolator.get(distanceMeters);
        double flightTime = 2.1;
        
        SmartDashboard.putNumber("Superstructure/Interpolated Ferry RPM", targetRPM);
        SmartDashboard.putNumber("Superstructure/Interpolated Ferry TOF", flightTime);

        return new InterpolatedFerryInfo(
            targetRPM, 
            flightTime
        );
    }    
}
