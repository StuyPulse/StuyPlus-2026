/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.shooter;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Gains.Swerve;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.stuypulse.robot.util.shooter.InterpolationCalculator.InterpolatedShotInfo;

public final class ShotCalculator {
    public static final double g = 9.81;

    // public record StationarySolution(
    //     Rotation2d targetHoodAngle,
    //     double targetRPM,
    //     double flightTimeSeconds) {
    // }

    // calculates the launch angle for a stationary robot to shoot into the pose
    // public static StationarySolution solveBallisticWithSpeed(
    //     Pose3d shooterPose,
    //     Pose3d targetPose,
    //     double targetRPM) {

    //     Translation3d s = shooterPose.getTranslation();
    //     Translation3d t = targetPose.getTranslation();

    //     double dx = t.getX() - s.getX();
    //     double dy = t.getY() - s.getY();
    //     double dz = t.getZ() - s.getZ();

    //     double d = Math.hypot(dx, dy);

    //     // convert to rad/s and multiply by radius to get m/s
    //     double launchSpeed = targetRPM * (2*Math.PI / 60) * Constants.HoodedShooter.Shooter.FLYWHEEL_RADIUS;
    //     double v2 = launchSpeed * launchSpeed;

    //     double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);

    //     if (discriminant < 0) {
    //         return new StationarySolution(Rotation2d.kZero, 0);
    //     }

    //     // LOW-ARC solution (use + for high arc)
    //     double tanTheta = (v2 - Math.sqrt(discriminant)) / (g * d);

    //     double launchAngle = Math.atan(tanTheta);

    //     double v_x = launchSpeed * Math.cos(launchAngle);
    //     double time = d / v_x;

    //     return new StationarySolution(Rotation2d.fromRadians(launchAngle), time);
    // }

    public record SOTMSolution(
        Pose2d virtualPose,
        double flightTime,
        double targetRPM) {
    }

    public static SOTMSolution solveShootOnTheMove(
        Pose2d robotPose,
        ChassisSpeeds fieldRelativeSpeeds,
        int maxIterations,
        double timeTolerance) {

        InterpolatedShotInfo sol = InterpolationCalculator.interpolateShotInfo();
        Pose2d hubPose = Field.getHubPose();
        
        double t_guess = sol.flightTimeSeconds();
        
        Pose2d virtualPose = hubPose;

             
        for (int i = 0; i < maxIterations; i++) {

            double dx = fieldRelativeSpeeds.vxMetersPerSecond * t_guess;
            double dy = fieldRelativeSpeeds.vyMetersPerSecond * t_guess;

            virtualPose = new Pose2d(
                hubPose.getX() - dx,
                hubPose.getY() - dy,
                hubPose.getRotation());

            InterpolatedShotInfo newSol = InterpolationCalculator.interpolateShotInfo(virtualPose);

            if (Math.abs(newSol.flightTimeSeconds() - t_guess) < timeTolerance) {
                break;
            }

            t_guess = newSol.flightTimeSeconds();

            sol = newSol;
        }

        return new SOTMSolution(
            virtualPose,
            sol.flightTimeSeconds(),
            sol.targetRPM()
        );
    }

    public static SOTMSolution solveFerryOnTheMove(
        Pose2d robotPose,
        ChassisSpeeds fieldRelativeSpeeds,
        int maxIterations,
        double timeTolerance) {

        InterpolatedShotInfo sol = InterpolationCalculator.interpolateShotInfo();
        Supplier<Pose2d> ferryZone = () -> Field.getFerryZonePose(robotPose.getTranslation());
        
        double t_guess = sol.flightTimeSeconds();
        
        Pose2d virtualPose = ferryZone.get();

        for (int i = 0; i < maxIterations; i++) {

            double dx = fieldRelativeSpeeds.vxMetersPerSecond * t_guess;
            double dy = fieldRelativeSpeeds.vyMetersPerSecond * t_guess;

            virtualPose = new Pose2d(
                ferryZone.get().getX() - dx,
                ferryZone.get().getY() - dy,
                ferryZone.get().getRotation());

            InterpolatedShotInfo newSol = InterpolationCalculator.interpolateFerryingRPM().get();

            if (Math.abs(newSol.flightTimeSeconds() - t_guess) < timeTolerance) {
                break;
            }

            t_guess = newSol.flightTimeSeconds();

            sol = newSol;
        }

        return new SOTMSolution(
            virtualPose,
            sol.flightTimeSeconds(),
            sol.targetRPM()
        );
    }

    public static double calculateShootingRPM() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        return solveShootOnTheMove(swerve.getPose(), swerve.getChassisSpeeds(), 5, 0.02).targetRPM();
    }

    public static double calculateFerryingRPM() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        return solveFerryOnTheMove(swerve.getPose(), swerve.getChassisSpeeds(), 5, 0.02).targetRPM();
    }
}