/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.intake.IntakeSetNinety;
import com.stuypulse.robot.commands.intake.IntakeSetZero;
import com.stuypulse.robot.commands.intake.IntakeSetZeroAtBottom;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
        SmartDashboard.putData("Intake/Set Pivot Ninety", new IntakeSetNinety());
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState {
        IDLE(Settings.Intake.Pivot.IDLE_ANGLE, 0), // (rollers do not run),
        DOWN(Settings.Intake.Pivot.DOWN_ANGLE, 0),
        INTAKE(Settings.Intake.Pivot.DOWN_ANGLE, Settings.Intake.Roller.INTAKE_DUTY_CYCLE), // (sucks in the balls) [pivot down, rollers running]
        OUTTAKE(Settings.Intake.Pivot.DOWN_ANGLE, Settings.Intake.Roller.OUTTAKE_DUTY_CYCLE), // (trips the balls out) [pivot down, rollers running reverse]
        AGITATE(Settings.Intake.Pivot.AGITATE_UP_ANGLE, 0),
        DIGEST(Settings.Intake.Pivot.DIGEST_ANGLE, 0),
        HOMING_DOWN(Settings.Intake.Pivot.DOWN_ANGLE, 0);

        private Angle targetAngle;
        private double targetDutyCycle;

        private IntakeState(Angle targetAngle, double targetDutyCycle) {
            this.targetAngle = targetAngle;
            this.targetDutyCycle = targetDutyCycle;
        }

        public Angle getTargetAngle() {
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
    public abstract Angle getRelativePosition();
    public abstract void setPivotZero(Angle angle);

    public boolean atTargetAngle() {
        return getRelativePosition().minus(getState().getTargetAngle()).abs(Rotations) < Settings.Intake.Pivot.ANGLE_TOLERANCE.in(Rotations);
    };
    public boolean isPivotAboveThreshold() {
        return getRelativePosition().gt(Settings.Intake.Pivot.PUSHDOWN_THRESHOLD);
    }

    // Roller Commands
    // public abstract AngularVelocity getRollerRPM();

    protected abstract void stopMotors();

    //Sysid
    public abstract SysIdRoutine getIntakeSysIdRoutine();
    public abstract void setPivotVoltageOverride(Voltage voltage);

    @Override
    public void periodic() {
        // RobotVisualizer.getInstance().updateIntake(Radians.of(getRelativePosition().in(Radians)), getRollerRPM());

        final IntakeState currentState = getState();
        // Logging
        SmartDashboard.putString("Intake/State", currentState.name());
        SmartDashboard.putString("States/Intake", currentState.name());

        SmartDashboard.putNumber("Intake/Pivot/Target Angle", currentState.getTargetAngle().in(Degrees));
        SmartDashboard.putNumber("Intake/Pivot/Current Angle", getRelativePosition().in(Degrees));
        SmartDashboard.putBoolean("Intake/Pivot/At Target Angle", atTargetAngle());
        SmartDashboard.putBoolean("Intake/Pivot/Above Threshold", isPivotAboveThreshold());
        SmartDashboard.putNumber("Intake/Rollers/Target Duty Cycle", currentState.getTargetDutyCycle());
        //SmartDashboard.putNumber("Intake/Rollers/RPM", getRollerRPM());
    }
}