/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.intake.IntakeSeedPivotDeployed;
import com.stuypulse.robot.commands.intake.IntakeSeedPivotNinety;
import com.stuypulse.robot.commands.intake.IntakeSeedPivotStowed;
import com.stuypulse.robot.constants.Settings;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.*;
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
        SmartDashboard.putData("Intake/Seed Pivot Angle Stowed", new IntakeSeedPivotStowed());
        SmartDashboard.putData("Intake/Set Pivot Angle Deployed", new IntakeSeedPivotDeployed());
        SmartDashboard.putData("Intake/Seed Pivot Angle 90", new IntakeSeedPivotNinety());
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState {

        /** The intake is stowed and rollers are off. */
        IDLE(Settings.Intake.Pivot.STOW_ANGLE, 0),
        /** The intake is deployed but rollers are off. */
        DOWN(Settings.Intake.Pivot.DEPLOY_ANGLE, 0),
        /** The intake is deployed and rollers are running to take in gamepieces. */
        INTAKE(Settings.Intake.Pivot.DEPLOY_ANGLE, Settings.Intake.Roller.INTAKE_DUTY_CYCLE),
        /**
         * The intake is deployed and rollers are running in reverse to expel
         * gamepieces.
         */
        OUTTAKE(Settings.Intake.Pivot.DEPLOY_ANGLE, Settings.Intake.Roller.OUTTAKE_DUTY_CYCLE),
        /**
         * The intake is brought up repeatedly to an angle between stowed and deployed
         * to dislodge
         * gamepieces. Rollers do not run.
         */
        AGITATE(Settings.Intake.Pivot.AGITATE_UP_ANGLE, 0),
        /**
         * The intake is brought up once to an angle between stowed and deployed to
         * dislodge gamepieces.
         * Rollers do not run.
         */
        DIGEST(Settings.Intake.Pivot.DIGEST_ANGLE, 0),
        /** The intake is pushed against the bumpers to re-zero the pivot. */
        HOMING_DOWN(Settings.Intake.Pivot.DEPLOY_ANGLE, 0);

        /** The target angle of the intake pivot. */
        private Angle targetAngle;

        /** The target percentage of voltage of the intake rollers. */
        private double targetDutyCycle;

        /**
         * Constructs an IntakeState with its target values.
         *
         * @param targetAngle     In any unit, the target position of the intake pivot
         * @param targetDutyCycle In any unit, the target percentage of voltage of the
         *                        intake rollers
         */
        private IntakeState(Angle targetAngle, double targetDutyCycle) {
            this.targetAngle = targetAngle;
            this.targetDutyCycle = targetDutyCycle;
        }

        /**
         * Gets the target position of the pivot.
         *
         * @return the target angle
         */
        public Angle getTargetAngle() {
            return targetAngle;
        }

        /**
         * Gets the target position of the pivot.
         *
         * @return the target percentage of voltage of the intake rollers
         */
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

    public abstract void seedPivotAngle(Angle angle);

    public boolean atTargetAngle() {
        return getRelativePosition().minus(getState().getTargetAngle())
                .abs(Rotations) < Settings.Intake.Pivot.ANGLE_TOLERANCE.in(Rotations);
    }

    public boolean isPivotAboveThreshold() {
        return getRelativePosition().gt(Settings.Intake.Pivot.PUSHDOWN_THRESHOLD);
    }

    // Roller Commands
    public abstract AngularVelocity getRollerVelocity();

    protected abstract void stopMotors();

    // Sysid
    public abstract SysIdRoutine getIntakeSysIdRoutine();

    public abstract void setPivotVoltageOverride(Voltage voltage);

    @Override
    public void periodic() {
        // RobotVisualizer.getInstance().updateIntake(Radians.of(getRelativePosition().in(Radians)),
        // getRollerRPM());
        final IntakeState currentState = getState();
        // Logging
        DogLog.log("Intake/State", currentState.name());
        DogLog.log("States/Intake", currentState.name());
        DogLog.log("Intake/Pivot/Target Angle", currentState.getTargetAngle().in(Degrees));
        DogLog.log("Intake/Pivot/Current Angle", getRelativePosition().in(Degrees));
        DogLog.log("Intake/Pivot/At Target Angle", atTargetAngle());
        DogLog.log("Intake/Pivot/Above Threshold", isPivotAboveThreshold());
        DogLog.log("Intake/Rollers/Target Duty Cycle", currentState.getTargetDutyCycle());
        DogLog.log("Intake/Rollers/RPM", getRollerVelocity().in(RPM));
    }
}
