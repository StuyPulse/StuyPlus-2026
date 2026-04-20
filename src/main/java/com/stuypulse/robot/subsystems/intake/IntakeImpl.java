package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeImpl extends Intake {
    private final TalonFX intakePivotMotor;
    private final TalonFX intakeRollerMotorLeft;
    private final TalonFX intakeRollerMotorRight;

    private final PositionTorqueCurrentFOC pivotController;
    private final VoltageOut homingController;
    private final TorqueCurrentFOC pushdownController;
    private final DutyCycleOut rollerController;
    private final Follower followerController;

    private final BooleanSupplier pivotStalling;
    private Optional<Voltage> pivotVoltageOverride;

    private final BStream leftRollerStalling;
    private final BStream rightRollerStalling;

    public IntakeImpl() {
        intakePivotMotor = new TalonFX(Ports.Intake.MOTOR_INTAKE_PIVOT, Settings.CANIVORE);
        Motors.Intake.PIVOT_CONFIG.configure(intakePivotMotor);
        intakePivotMotor.setPosition(Settings.Intake.Pivot.INITIAL_ANGLE.getRotations());

        intakeRollerMotorLeft = new TalonFX(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, Settings.CANIVORE); // leader
        intakeRollerMotorRight = new TalonFX(Ports.Intake.MOTOR_INTAKE_ROLLER_RIGHT, Settings.CANIVORE);

        Motors.Intake.LEFT_ROLLER_CONFIG.configure(intakeRollerMotorLeft);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(intakeRollerMotorRight);

        pivotController = new PositionTorqueCurrentFOC(getState().getTargetAngle().getRotations());
        homingController = new VoltageOut(Settings.Intake.Pivot.HOMING_DOWN_VOLTAGE).withEnableFOC(true);
        pushdownController = new TorqueCurrentFOC(Settings.Intake.Pivot.PUSHDOWN_CURRENT.getAsDouble());
        rollerController = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);
        followerController = new Follower(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, MotorAlignmentValue.Opposed);

        intakeRollerMotorRight.setControl(followerController);

        pivotStalling = () -> intakePivotMotor.getStatorCurrent().getValueAsDouble() > Settings.Intake.Pivot.STALL_CURRENT.in(Amps);
        leftRollerStalling = BStream
                .create(() -> intakeRollerMotorLeft.getStatorCurrent()
                        .getValueAsDouble() > Settings.Intake.Roller.STALL_CURRENT.in(Amps))
                .filtered(new BDebounce.Both(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds)));
        rightRollerStalling = BStream
                .create(() -> intakeRollerMotorRight.getStatorCurrent().getValueAsDouble() > Settings.Intake.Roller.STALL_CURRENT.in(Amps))
                    .filtered(new BDebounce.Both(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds)));

        pivotVoltageOverride = Optional.empty();
    }

    public void setPivotVoltageOverride(Voltage voltage) {
        this.pivotVoltageOverride = Optional.of(voltage);
    }

    /**********************/
    /*** Pivot Commands ***/
    /**********************/
    @Override
    public Rotation2d getRelativePosition() {
        return Rotation2d.fromRotations(intakePivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setPivotZeroAtBottom() {
        intakePivotMotor.setPosition(Settings.Intake.Pivot.DOWN_ANGLE.getRotations());
    }

    @Override
    public void setPivotZero() {
        intakePivotMotor.setPosition(0);
    }

    private boolean pivotStalling() {
        return pivotStalling.getAsBoolean();
    }

    /***********************/
    /*** Roller Commands ***/
    /***********************/
    @Override
    public double getRollerRPM() {
        return intakeRollerMotorRight.getVelocity().getValue().in(RPM);
    }

    private boolean leftRollerStalling() {
        return leftRollerStalling.get();
    }

    private boolean rightRollerStalling() {
        return rightRollerStalling.get();
    }

    @Override
    protected void stopMotors() {
        intakePivotMotor.stopMotor();
        intakeRollerMotorLeft.stopMotor();
        intakeRollerMotorRight.stopMotor();
        intakeRollerMotorRight.setControl(followerController); // re-add the follow control after stopMotor removes it
    }

    @Override
    public void periodic() {
        if (!EnabledSubsystems.INTAKE.get()) {
            stopMotors();
            return;
        }

        if (pivotVoltageOverride.isPresent()) {
            intakePivotMotor.setVoltage(pivotVoltageOverride.get().in(Volts));
            return;
        }

        // Input

        final boolean pivotAboveThreshold = isPivotAboveThreshold();

        final boolean pivotStalling = pivotStalling();

        final IntakeState currentState = getState();

        // State

        if (currentState == IntakeState.HOMING_DOWN && pivotStalling) {
            setPivotZeroAtBottom();
            setState(IntakeState.DOWN);
        }

        if ((currentState == IntakeState.DOWN) && pivotStalling) {
            setPivotZeroAtBottom();
        }

        // Output

        final ControlRequest pivotControl = switch (currentState) {
            case INTAKE, OUTTAKE, DOWN -> {
                if (pivotAboveThreshold) {
                    // wait until pivot reaches the bottom to apply pushdown
                    yield pushdownController;
                } else {
                    yield pivotController.withPosition(currentState.getTargetAngle().getRotations());
                }
            }
            case HOMING_DOWN -> homingController;
            default -> pivotController.withPosition(currentState.getTargetAngle().getRotations());
        };

        final DutyCycleOut rollerControl = rollerController.withOutput(currentState.getTargetDutyCycle());

        // Apply

        
        intakePivotMotor.setControl(pivotControl);
        intakeRollerMotorLeft.setControl(rollerControl);

        // Logging
        SmartDashboard.putNumber("Intake/Pivot/Stator Current", intakePivotMotor.getStatorCurrent().getValueAsDouble()); // all current measured in amps
        SmartDashboard.putNumber("Intake/Pivot/Supply Current", intakePivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Voltage", intakePivotMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Rollers/Left Current", intakeRollerMotorLeft.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Right Current", intakeRollerMotorRight.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Left Voltage", intakeRollerMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Right Voltage", intakeRollerMotorRight.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Rollers/Left Stalling", leftRollerStalling());
        SmartDashboard.putBoolean("Intake/Rollers/Right Stalling", rightRollerStalling());

        super.periodic();
    }

    public SysIdRoutine getIntakeSysIdRoutine() {
        return SysId.getRoutine(Settings.Intake.Pivot.RAMP_RATE,
                Settings.Intake.Pivot.STEP_VOLTAGE,
                "Intake",
                voltage -> setPivotVoltageOverride(voltage),
                () -> intakePivotMotor.getPosition().getValue(),
                () -> intakePivotMotor.getVelocity().getValue(),
                () -> intakePivotMotor.getMotorVoltage().getValue(),
                getInstance());
    }
}