/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.intake.IntakeSetZero;
import com.stuypulse.robot.commands.intake.IntakeSetZeroAtBottom;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import dev.doglog.DogLog;

public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

    private IntakeState state;

    static {
        if (Robot.isReal()) {
            instance = new IntakeImpl();
        } else {
            instance = new IntakeSim();
        }
        // Elastic Commands
        SmartDashboard.putData("Intake/Set Pivot 0", new IntakeSetZero());
        SmartDashboard.putData("Intake/Set Pivot 0 at Bottom", new IntakeSetZeroAtBottom());
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState {

        // (rollers do not run),
        IDLE(Settings.Intake.Pivot.IDLE_ANGLE, 0),
        AGITATE(Settings.Intake.Pivot.AGITATE_UP_ANGLE, 0),
        // (sucks in the balls) [pivot down, rollers running]
        INTAKE(Settings.Intake.Pivot.DOWN_ANGLE, Settings.Intake.Roller.INTAKE_DUTY_CYCLE),
        // (trips the balls out) [pivot down, rollers running reverse]
        OUTTAKE(Settings.Intake.Pivot.DOWN_ANGLE, Settings.Intake.Roller.OUTTAKE_DUTY_CYCLE),
        DOWN(Settings.Intake.Pivot.DOWN_ANGLE, 0),
        HOMING_DOWN(Settings.Intake.Pivot.DOWN_ANGLE, 0);

        private Rotation2d targetAngle;

        private double targetDutyCycle;

        private IntakeState(Rotation2d targetAngle, double targetDutyCycle) {
            this.targetAngle = targetAngle;
            this.targetDutyCycle = targetDutyCycle;
        }

        public Rotation2d getTargetAngle() {
            return targetAngle;
        }

        public double getTargetDutyCycle() {
            return targetDutyCycle;
        }
    }

    protected Intake() {
        this.state = IntakeState.IDLE;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }

    // Pivot Commands
    public abstract Rotation2d getRelativePosition();

    public abstract void setPivotZero();

    public abstract void setPivotZeroAtBottom();

    public boolean atTargetAngle() {
        return Math.abs((getRelativePosition().getRotations()) - getState().getTargetAngle().getRotations()) < Settings.Intake.Pivot.ANGLE_TOLERANCE.getRotations();
    }

    public boolean isPivotAboveThreshold() {
        return getRelativePosition().getDegrees() > Settings.Intake.Pivot.PUSHDOWN_THRESHOLD.getDegrees();
    }

    // Roller Commands
    public abstract double getRollerRPM();

    protected abstract void stopMotors();

    // Sysid
    public abstract SysIdRoutine getIntakeSysIdRoutine();

    public abstract void setPivotVoltageOverride(Voltage voltage);

    @Override
    public void periodic() {
        RobotVisualizer.getInstance().updateIntake(Radians.of(getRelativePosition().getRadians()), RPM.of(getRollerRPM()));
        final IntakeState currentState = getState();
        // Logging
        DogLog.log("Intake/State", currentState.name());
        DogLog.log("States/Intake", currentState.name());
        DogLog.log("Intake/Pivot/Target Angle", currentState.getTargetAngle().getDegrees());
        DogLog.log("Intake/Pivot/Current Angle", getRelativePosition().getDegrees());
        DogLog.log("Intake/Pivot/At Target Angle", atTargetAngle());
        DogLog.log("Intake/Pivot/Above Threshold", isPivotAboveThreshold());
        DogLog.log("Intake/Rollers/Target Duty Cycle", currentState.getTargetDutyCycle());
        DogLog.log("Intake/Rollers/RPM", getRollerRPM());
    }
}
