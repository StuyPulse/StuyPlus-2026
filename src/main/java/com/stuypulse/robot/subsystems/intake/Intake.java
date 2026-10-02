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
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIOPivotOutputMode;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIORollerOutputMode;
import com.stuypulse.robot.util.FullSubsystem;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends FullSubsystem {
    private static final Intake instance;

    private final BooleanSupplier leftRollerStalling;
    private final BooleanSupplier rightRollerStalling;
    private final BooleanSupplier pivotStalling;

    private final Debouncer leftRollerDebouncer;
    private final Debouncer rightRollerDebouncer;

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
    private final IntakeIOOutputs outputs;
    @AutoLogOutput(key = "States/Intake")
    private IntakeState state;

    private Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputsAutoLogged();
        this.outputs = new IntakeIOOutputs();
        this.state = IntakeState.IDLE;

        this.leftRollerStalling = () -> inputs.leftRollerMotorInputs.statorCurrent.gt(Settings.Intake.Roller.STALL_CURRENT);
        this.rightRollerStalling = () -> inputs.rightRollerMotorInputs.statorCurrent.gt(Settings.Intake.Roller.STALL_CURRENT);
        this.pivotStalling = () -> inputs.pivotMotorInputs.statorCurrent.gt(Settings.Intake.Pivot.STALL_CURRENT);

        this.leftRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), Debouncer.DebounceType.kBoth);
        this.rightRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), Debouncer.DebounceType.kBoth);
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
        return inputs.pivotMotorInputs.position;
    }

    @AutoLogOutput(key = "Intake/Left Roller Stalling")
    public boolean isLeftRollerStalling() {
        return leftRollerDebouncer.calculate(leftRollerStalling.getAsBoolean());
    }

    @AutoLogOutput(key = "Intake/Right Roller Stalling")
    public boolean isRightRollerStalling() {
        return rightRollerDebouncer.calculate(rightRollerStalling.getAsBoolean());
    }

    @AutoLogOutput(key = "Intake/Pivot Stalling")
    public boolean isPivotStalling() {
        return pivotStalling.getAsBoolean();
    }

    @AutoLogOutput(key = "Intake/Pivot/atTargetAngle")
    public boolean atTargetAngle() {
        return inputs.pivotMotorInputs.position.minus(getState().getTargetAngle())
                .abs(Rotations) < Settings.Intake.Pivot.ANGLE_TOLERANCE.in(Rotations);
    }

    @AutoLogOutput(key = "Intake/Pivot/aboveThreshold")
    public boolean isPivotAboveThreshold() {
        return inputs.pivotMotorInputs.position.gt(Settings.Intake.Pivot.PUSHDOWN_THRESHOLD);
    }

    /*********************/
    /** Pivot Controls ***/
    /*********************/
  
    private void setPivotPosition(Angle position, int gainsSlot) {
        outputs.pivot.outputMode = IntakeIOPivotOutputMode.POSITION;
        outputs.pivot.position = position;
        outputs.pivot.positionGainsSlot = gainsSlot;
    }

    private void setPivotPushdown(Current current) {
        outputs.pivot.outputMode = IntakeIOPivotOutputMode.PUSHDOWN;
        outputs.pivot.pushdown = current;
    }
    
    private void setPivotHoming(Voltage voltage) {
        outputs.pivot.outputMode = IntakeIOPivotOutputMode.HOMING;
        outputs.pivot.homing = voltage; 
    }

    /*********************/
    /** Roller Control ***/
    /*********************/

    private void setRollerDutyCycle(double dutyCycle) {
        outputs.roller.outputMode = IntakeIORollerOutputMode.DUTY_CYCLE;
        outputs.roller.targetDutyCycle = dutyCycle;
    }

    /*********************/
    /** Pivot Commands ***/
    /*********************/

    public void seedPivotAngle(Angle angle) {
        io.seedPivotAngle(angle);
    }

    private void stopAllMotors() {
        outputs.pivot.outputMode = IntakeIOPivotOutputMode.STOP;
        outputs.roller.outputMode = IntakeIORollerOutputMode.STOP;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        final IntakeState currentState = getState();
    
        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            stopAllMotors();
            return;
        }

        if (inputs.limitSwitchHit) {
            io.seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
        }

        if (currentState == IntakeState.HOMING_DOWN && (isPivotStalling() || inputs.limitSwitchHit)) {
            io.seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
            setState(IntakeState.INTAKE);
        }

        if ((currentState == IntakeState.DOWN) && (isPivotStalling() || inputs.limitSwitchHit)) {
            io.seedPivotAngle(Settings.Intake.Pivot.DEPLOY_ANGLE);
        }

        switch (currentState) {
            case INTAKE, OUTTAKE, DOWN -> setPivotPushdown(Amps.of(Settings.Intake.Pivot.PUSHDOWN_CURRENT.get()));
            case HOMING_DOWN -> setPivotHoming(Settings.Intake.Pivot.HOMING_DOWN_VOLTAGE);
            case AGITATE, AGITATE_DOWN -> setPivotPosition(currentState.getTargetAngle(), 1);
            default -> setPivotPosition(currentState.getTargetAngle(), 0);
        }

        setRollerDutyCycle(currentState.getTargetDutyCycle());
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }
}
