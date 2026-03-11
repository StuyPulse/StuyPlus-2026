/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.shooter;

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
        Pose2d targetPose,
        ChassisSpeeds fieldRelativeSpeeds,
        int maxIterations,
        double timeTolerance) {

        /*
        Start with v_ball * flightTime = distanceToTargetPose.
        
        We know that v_ball = v_robot + v_shooter, so 
        (v_robot + v_shooter) * flightTime = distanceToTargetPose

        Rearranging, we can get
        (v_shooter) * flight_time = distanceToTargetPose - v_robot * flightTime

        So we can instead shoot at a virtual pose and treat the robot as stationary:
        distanceToVirtualPose = distanceToTargetPose - v_robot * flightTime
        (v_shooter) * flight_time = distanceToVirtualPose

        Looking at the first equation, we can find the virtual pose with the flight time, 
        but looking at the second equation, to get the flight time we need to solveBallisticWithSpeed()
        using the virtual pose, so we have a circular dependence.

        Thus, we can make an initial guess for the flight time: the flight time if the robot were stationary
        We want our guess to converge such that the left side equals the right side:
        (v_shooter) * t_guess = distanceToVirtualPose = distance - v_robot * t_guess, which would make t_guess = flightTime

        We do the right side first using our inital guess, and then update t_guess with a new guess by 
        calculating the flightTime to that virtualPose.

        The pose is that the flightTime converges within maxIterations.
        */
        

        // StationarySolution sol = solveBallisticWithSpeed(
        //     turretPose,
        //     targetPose,
        //     targetRPM
        // );

        InterpolatedShotInfo sol = InterpolationCalculator.interpolateShotInfo();

        
        double t_guess = sol.flightTimeSeconds();
        
        Pose2d virtualPose = targetPose;

             
        for (int i = 0; i < maxIterations; i++) {

            double dx = fieldRelativeSpeeds.vxMetersPerSecond * t_guess;
            double dy = fieldRelativeSpeeds.vyMetersPerSecond * t_guess;

            virtualPose = new Pose2d(
                targetPose.getX() - dx,
                targetPose.getY() - dy,
                targetPose.getRotation());

            // StationarySolution newSol = solveBallisticWithSpeed(
            //     turretPose,
            //     virtualPose,
            //     targetRPM
            // );

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

    public static double calculateShootingRPM() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        return solveShootOnTheMove(swerve.getPose(), Field.getHubPose(), swerve.getChassisSpeeds(), 5, 0.02).targetRPM();
    }

    public static double calculateFerryingRPM() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        return solveShootOnTheMove(swerve.getPose(), Field.getFerryZonePose(swerve.getPose().getTranslation()), swerve.getChassisSpeeds(), 5, 0.02).targetRPM();
    }
}