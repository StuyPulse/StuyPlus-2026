package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    private final DutyCycleOut rollerController;
    private final PositionVoltage pivotController;
    private final Follower followerController;

    private final BStream pivotStalling;
    private final BStream rollersStalling;
    private Optional<Double> pivotVoltageOverride;

    public IntakeImpl() {
        intakePivotMotor = new TalonFX(Ports.Intake.MOTOR_INTAKE_PIVOT, Settings.CANIVORE);
        Motors.Intake.PIVOT_CONFIG.configure(intakePivotMotor);
        intakePivotMotor.setPosition(Settings.Intake.PIVOT_INITIAL_ANGLE.getRotations());

        intakeRollerMotorLeft = new TalonFX(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, Settings.CANIVORE); // leader
        intakeRollerMotorRight = new TalonFX(Ports.Intake.MOTOR_INTAKE_ROLLER_RIGHT, Settings.CANIVORE);

        Motors.Intake.LEFT_ROLLER_CONFIG.configure(intakeRollerMotorLeft);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(intakeRollerMotorRight);

        rollerController = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);
        pivotController = new PositionVoltage(getState().getTargetAngle().getRotations()).withEnableFOC(true);
        followerController = new Follower(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, MotorAlignmentValue.Opposed);

        intakeRollerMotorRight.setControl(followerController);

        pivotStalling = BStream.create(
                () -> intakePivotMotor.getSupplyCurrent().getValueAsDouble() > Settings.Intake.PIVOT_STALL_CURRENT)
                .filtered(new BDebounce.Both(Settings.Intake.PIVOT_STALL_DEBOUNCE_SEC));
        rollersStalling = BStream
                .create(() -> intakeRollerMotorLeft.getSupplyCurrent()
                        .getValueAsDouble() > Settings.Intake.ROLLER_STALL_CURRENT)
                .filtered(new BDebounce.Both(Settings.Intake.ROLLER_STALL_DEBOUNCE_SEC));

        pivotVoltageOverride = Optional.empty();
    }

    @Override
    public Rotation2d getRelativePosition() {
        return Rotation2d.fromRotations(intakePivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public boolean atAngle() {
        return Math.abs(
                (getRelativePosition().getRotations())
                        - getState().getTargetAngle().getRotations()) < Settings.Intake.ANGLE_TOLERANCE.getRotations();
    }

    public void setPivotVoltageOverride(Optional<Double> pivotVoltageOverride) {
        this.pivotVoltageOverride = pivotVoltageOverride;
    }

    @Override
    public double getRollerRPM() {
        return intakeRollerMotorLeft.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    public void setPivotZero() {
        intakePivotMotor.setPosition(0);
    }

    @Override
    public void setPivotZeroAtBottom() {
        intakePivotMotor.setPosition(Rotation2d.fromDegrees(122).getRotations());
    }

    private boolean pivotStalling() {
        return pivotStalling.get();
    }

    private boolean rollersStalling() {
        return rollersStalling.get();
    }

    private void stopMotors() {
        intakePivotMotor.stopMotor();
        intakeRollerMotorLeft.stopMotor();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!EnabledSubsystems.INTAKE.get()) {
            stopMotors();
            return;
        }

        // Input

        final double pivotPosition = intakePivotMotor.getPosition().getValueAsDouble();
        final boolean pivotAboveThreshold = pivotPosition > Settings.Intake.PUSHDOWN_THRESHOLD.getRotations();

        final boolean pivotStalling = pivotStalling();

        final IntakeState currentState = getState();

        final boolean pushingDown = currentState == IntakeState.INTAKE ||
                currentState == IntakeState.OUTTAKE ||
                currentState == IntakeState.DOWN &&
                !pivotAboveThreshold;

        // State

        if (currentState == IntakeState.HOMING_UP && pivotStalling) {
            setPivotZero();
            setState(IntakeState.IDLE);
        }

        if (currentState == IntakeState.HOMING_DOWN && pivotStalling) {
            setPivotZeroAtBottom();
            setState(IntakeState.INTAKE);
        }

        if ((currentState == IntakeState.INTAKE || currentState == IntakeState.OUTTAKE || currentState == IntakeState.DOWN) && pivotStalling) {
            setPivotZeroAtBottom();
        }

        // if (rollersStalling()) {
        //     setState(IntakeState.DOWN);
        //     setPivotZeroAtBottom();
        // }

        // Output

        final ControlRequest pivotControl = switch (currentState) {
            case INTAKE, OUTTAKE, DOWN -> {
                if (pivotAboveThreshold) {
                    // yield pivotController.withPosition(currentState.getTargetAngle().getRotations());
                    yield new VoltageOut(Settings.Intake.PUSHDOWN_VOLTAGE); // wait until pivot reaches the bottom to apply pushdown
                } else {
                    yield pivotController.withPosition(currentState.getTargetAngle().getRotations());
                }
            }

            case HOMING_UP -> new VoltageOut(Settings.Intake.HOMING_UP_VOLTAGE);
            case HOMING_DOWN -> new VoltageOut(Settings.Intake.HOMING_DOWN_VOLTAGE);
            default -> pivotController.withPosition(currentState.getTargetAngle().getRotations());
        };

        final DutyCycleOut rollerControl = rollerController.withOutput(pivotAboveThreshold ? currentState.getTargetDutyCycle() : 0);

        // Apply

        if (pivotVoltageOverride.isPresent()) {
            intakePivotMotor.setVoltage(pivotVoltageOverride.get());
        } else {
            intakePivotMotor.setControl(pivotControl);
        }
        intakeRollerMotorLeft.setControl(rollerControl);

        // Log

        SmartDashboard.putNumber("Intake/Left Roller Current (amps)",
                intakeRollerMotorLeft.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Right Roller Current", intakeRollerMotorRight.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Left Roller Voltage", intakeRollerMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Right Roller Voltage", intakeRollerMotorRight.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Pushing Down", pushingDown);

        SmartDashboard.putNumber("Intake/Left Roller Duty Cycle", intakeRollerMotorLeft.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Right Roller Duty Cycle", intakeRollerMotorRight.getDutyCycle().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Pivot Current (amps)", intakePivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Voltage", intakePivotMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Pivot Angle (deg)", getRelativePosition().getDegrees());

        SmartDashboard.putBoolean("Intake/Pivot At Target Angle", atAngle());
    }

    public SysIdRoutine getIntakeSysIdRoutine() {
        return SysId.getRoutine(Settings.Intake.RAMP_RATE,
                Settings.Intake.STEP_VOLTAGE,
                "Intake",
                voltage -> setPivotVoltageOverride(Optional.of(voltage)),
                () -> intakePivotMotor.getPosition().getValueAsDouble(),
                () -> intakePivotMotor.getVelocity().getValueAsDouble(),
                () -> intakePivotMotor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }
}