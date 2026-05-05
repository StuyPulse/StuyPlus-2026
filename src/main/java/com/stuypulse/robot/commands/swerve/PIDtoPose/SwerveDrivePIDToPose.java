/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.PIDtoPose;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains.Swerve.Alignment;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.TranslationMotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;

public class SwerveDrivePIDToPose extends Command {
    
    private final CommandSwerveDrivetrain swerve;

    private final HolonomicController controller;
    private final Supplier<Pose2d> targetPose;

    private double maxVelocity;
    private double maxAcceleration;

    private boolean isMotionProfiled;

    private final BStream isAligned;
    private final IStream velocityError;

    private final FieldObject2d targetPose2d;

    private Number xTolerance;
    private Number yTolerance;
    private Number thetaTolerance;
    private Number maxVelocityWhenAligned;

    private VStream translationSetpoint;

    private BooleanSupplier canEnd;

    public SwerveDrivePIDToPose(Pose2d targetPose) {
        this(() -> targetPose);
    }
    
    public SwerveDrivePIDToPose(Supplier<Pose2d> targetPose) {
        swerve = CommandSwerveDrivetrain.getInstance();

        controller = new HolonomicController(
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD),
            new PIDController(Alignment.XY.kP, Alignment.XY.kI, Alignment.XY.kD),
            new AnglePIDController(Alignment.THETA.kP, Alignment.THETA.kI, Alignment.THETA.kD)
            .setSetpointFilter(new AMotionProfile(Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_VELOCITY, Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ANGULAR_ACCELERATION)));

            maxVelocity = Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_VELOCITY;
            maxAcceleration = Settings.Swerve.Alignment.Constraints.DEFAULT_MAX_ACCELERATION;


            isMotionProfiled = true;
            translationSetpoint = getNewTranslationSetpointGenerator();

            this.targetPose = targetPose;

            targetPose2d = Field.FIELD2D.getObject("Target Pose");

            isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE.in(Seconds)));

            velocityError = IStream.create(() -> new Translation2d(controller.getError().vxMetersPerSecond, controller.getError().vyMetersPerSecond).getNorm())
            .filtered(new LowPassFilter(0.05))
            .filtered(x -> Math.abs(x));

            xTolerance = Settings.Swerve.Alignment.Tolerances.X_TOLERANCE.in(Meters);
            yTolerance = Settings.Swerve.Alignment.Tolerances.Y_TOLERANCE.in(Meters);
            thetaTolerance = Settings.Swerve.Alignment.Tolerances.THETA_TOLERANCE.getRadians();
            maxVelocityWhenAligned = Settings.Swerve.Alignment.Tolerances.MAX_VELOCITY_WHEN_ALIGNED.in(MetersPerSecond);

            canEnd = () -> true;

            addRequirements(swerve);
    }
    
    public SwerveDrivePIDToPose withTolerance(double x, double y, Rotation2d theta) {
        xTolerance = x;
        yTolerance = y;
        thetaTolerance = theta.getRadians();
        return this;
    }
    
    public SwerveDrivePIDToPose withTranslationalConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        return this;
    }
    
    public SwerveDrivePIDToPose withoutMotionProfile() {
        this.isMotionProfiled = false;
        return this;
    }

    public SwerveDrivePIDToPose withCanEnd(BooleanSupplier canEnd){
        this.canEnd = canEnd;
        return this;
    }

    private VStream getNewTranslationSetpointGenerator(){
        if(!isMotionProfiled){
            return VStream.create(() -> new Vector2D(targetPose.get().getTranslation()));
        }
        else{
            return VStream.create(() -> new Vector2D(targetPose.get().getTranslation()))
            .filtered(new TranslationMotionProfile(
                this.maxVelocity,
                this.maxAcceleration,
                new Vector2D(swerve.getPose().getTranslation()),
                Vector2D.kOrigin));
        }
    }

    @Override
    public void initialize(){
        translationSetpoint = getNewTranslationSetpointGenerator();        
    }
    
    public boolean isAlignedX(){
        return Math.abs(targetPose.get().getX() - swerve.getPose().getX()) < xTolerance.doubleValue();
    }

    public boolean isAlignedY(){
        return Math.abs(targetPose.get().getY() - swerve.getPose().getY()) < yTolerance.doubleValue();
    }

    public boolean isAlignedTheta(){
        return Math.abs(targetPose.get().getRotation().minus(swerve.getPose().getRotation()).getRadians()) < thetaTolerance.doubleValue();
    }

    public boolean isAligned(){
        return isAlignedX() && isAlignedY() && isAlignedTheta() && velocityError.get() < maxVelocityWhenAligned.doubleValue();
    }

    @Override
    public void execute() {
        targetPose2d.setPose(Robot.isBlue() ? targetPose.get() : Field.transformToOppositeAlliance(targetPose.get()));
        
        controller.update(new Pose2d(translationSetpoint.get().getTranslation2d(), targetPose.get().getRotation()), swerve.getPose());

        swerve.setControl(swerve.getRobotCentricSwerveRequest()
        .withVelocityX(controller.getOutput().vxMetersPerSecond)
        .withVelocityY(controller.getOutput().vyMetersPerSecond)
        .withRotationalRate(controller.getOutput().omegaRadiansPerSecond));

        SmartDashboard.putNumber("Alignment/Target x", targetPose.get().getX());
        SmartDashboard.putNumber("Alignment/Target y", targetPose.get().getY());
        SmartDashboard.putNumber("Alignment/Target Angle", targetPose.get().getRotation().getDegrees());

        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative X (m/s)", controller.getOutput().vxMetersPerSecond);
        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative Y (m/s)", controller.getOutput().vyMetersPerSecond);
        SmartDashboard.putNumber("Alignment/Target Angular Velocity (rad/s)", controller.getOutput().omegaRadiansPerSecond);

        SmartDashboard.putBoolean("Alignment/Is Aligned", isAligned());
        SmartDashboard.putBoolean("Alignment/Is Aligned X", isAlignedX());
        SmartDashboard.putBoolean("Alignment/Is Aligned Y", isAlignedY());
        SmartDashboard.putBoolean("Alignment/Is Aligned Theta", isAlignedTheta());
    }

    @Override
    public boolean isFinished(){
        return isAligned.get() && canEnd.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted){
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        Field.clearFieldObject(targetPose2d);
    }
}
