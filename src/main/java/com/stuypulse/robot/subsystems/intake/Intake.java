/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.intake.IntakeSeedPivotDeployed;
import com.stuypulse.robot.commands.intake.IntakeSeedPivotNinety;
import com.stuypulse.robot.commands.intake.IntakeSeedPivotStowed;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.simulation.RobotVisualizer;

import dev.doglog.DogLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {
    private static final Intake instance;

    static {
        if (Robot.isReal()) {
            instance = new Intake(new IntakeIOTalonFX());
        } else {
            instance = new Intake(new IntakeIOSim());
        }
        // Elastic Commands
        SmartDashboard.putData("Intake/Seed Pivot Angle Stowed", new IntakeSeedPivotStowed());
        SmartDashboard.putData("Intake/Set Pivot Angle Deployed", new IntakeSeedPivotDeployed());
        SmartDashboard.putData("Intake/Seed Pivot Angle 90", new IntakeSeedPivotNinety());
    }

    public static Intake getInstance() {
        return instance;
    }

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;
    private IntakeState state;

    protected Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputsAutoLogged();
        this.state = IntakeState.IDLE;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }

    /** Enum representing the different possible states of the intake. */
    public enum IntakeState {

        AGITATE_DOWN(Settings.Intake.Pivot.AGITATE_DOWN_ANGLE, Settings.Intake.Roller.INTAKE_DUTY_CYCLE),
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
        AGITATE(Settings.Intake.Pivot.AGITATE_UP_ANGLE, Settings.Intake.Roller.INTAKE_DUTY_CYCLE),
        
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

    public Angle getRelativePosition() {
        return inputs.pivotPosition;
    }

    public boolean atTargetAngle() {
        return inputs.pivotPosition.minus(getState().getTargetAngle())
                .abs(Rotations) < Settings.Intake.Pivot.ANGLE_TOLERANCE.in(Rotations);
    }

    public boolean isPivotAboveThreshold() {
        return inputs.pivotPosition.gt(Settings.Intake.Pivot.PUSHDOWN_THRESHOLD);
    }

    // Sysid
    private Optional<Voltage> pivotVoltageOverride = Optional.empty();

    public void setPivotVoltageOverride(Voltage voltage) {
        this.pivotVoltageOverride = Optional.of(voltage);
    };

    public SysIdRoutine getIntakeSysIdRoutine() {
        return SysId.getRoutine(
                Settings.Intake.Pivot.RAMP_RATE,
                Settings.Intake.Pivot.STEP_VOLTAGE,
                "Intake",
                this::setPivotVoltageOverride,
                () -> inputs.pivotPosition,
                () -> inputs.pivotVelocity,
                () -> inputs.pivotVoltage,
                getInstance());
    }

    public void seedPivotAngle(Angle angle) {
        io.seedPivotAngle(angle);
    }

    @Override
    public void periodic() {
        final IntakeState currentState = getState();
    
        if (Settings.EnabledSubsystems.INTAKE.get()) {
            // roller
            io.setRollerDutyCycle(currentState.getTargetDutyCycle());

            // pivot
            if (pivotVoltageOverride.isEmpty()) {
                switch (currentState) {
                    case INTAKE, OUTTAKE, DOWN:
                        if (isPivotAboveThreshold()) {
                            io.setPivotPushdown(Amps.of(Settings.Intake.Pivot.PUSHDOWN_CURRENT.get()));
                        } else {
                            io.setPivotPosition(currentState.getTargetAngle());
                        };
                        break;
                    case HOMING_DOWN:
                        io.setPivotHoming(Settings.Intake.Pivot.HOMING_DOWN_VOLTAGE);
                        break;
                    case AGITATE, AGITATE_DOWN:
                        io.setPivotPosition(currentState.getTargetAngle());
                        break;
                    default: io.setPivotPosition(currentState.getTargetAngle());
                };
            } else {
                io.setPivotHoming(pivotVoltageOverride.get());
            }
        } else {
            io.stopAllMotors();
        }
        io.updateInputs(inputs);
        RobotVisualizer.getInstance().updateIntake(inputs.pivotPosition, inputs.rollerVelocity);

        DogLog.log("Intake/State", currentState.name());
    }
}
