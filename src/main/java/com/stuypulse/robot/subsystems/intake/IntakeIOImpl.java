package com.stuypulse.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.LoggedSignals;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import edu.wpi.first.wpilibj.DigitalInput;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class IntakeIOImpl implements IntakeIO {
    private final DigitalInput pivotLimitSwitch;

    private final TalonFX pivotMotor;

    private final TalonFX rollerMotorLeft; // Leader
    private final TalonFX rollerMotorRight; // Follower

    private final PositionTorqueCurrentFOC positionController;
    private final VoltageOut homingController;
    private final TorqueCurrentFOC pushdownController;
    private final DutyCycleOut rollerController;
    private final Follower rollerFollower;

    private final LoggedSignals pivotSignals;
    private final LoggedSignals rollerSignals;

    private final BooleanSupplier pivotStalling;
    private final BooleanSupplier leftRollerStalling;
    private final BooleanSupplier rightRollerStalling;

    private final Debouncer leftRollerDebouncer;
    private final Debouncer rightRollerDebouncer;

    public IntakeIOImpl() {
        this.pivotLimitSwitch = new DigitalInput(Ports.Intake.PIVOT_LIMIT_SWITCH);
        
        this.pivotMotor = new TalonFX(Ports.Intake.INTAKE_PIVOT_MOTOR, Settings.CANBUS);
        Motors.Intake.PIVOT_CONFIG.configure(pivotMotor);

        // zero it at the up position
        pivotMotor.setPosition(Settings.Intake.Pivot.INITIAL_ANGLE);
        this.pivotSignals = new LoggedSignals(LoggedSignals.SignalLocation.CANIVORE, "Intake/Pivot/",
                pivotMotor.getSupplyCurrent(),
                pivotMotor.getStatorCurrent(),
                pivotMotor.getVelocity());

        this.rollerMotorLeft = new TalonFX(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, Settings.CANBUS);
        Motors.Intake.LEFT_ROLLER_CONFIG.configure(rollerMotorLeft);

        this.rollerMotorRight = new TalonFX(Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT, Settings.CANBUS);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(rollerMotorRight);

        this.rollerSignals = new LoggedSignals(LoggedSignals.SignalLocation.CANIVORE, "Intake/Roller/").withMotor(
                "Intake Roller Left",
                rollerMotorLeft.getSupplyCurrent(),
                rollerMotorLeft.getStatorCurrent(),
                rollerMotorLeft.getVelocity())
                .withMotor(
                        "Intake Roller Right",
                        rollerMotorRight.getSupplyCurrent(),
                        rollerMotorRight.getStatorCurrent(),
                        rollerMotorRight.getVelocity());

        this.positionController = new PositionTorqueCurrentFOC(0.0);
        this.homingController = new VoltageOut(Settings.Intake.Pivot.HOMING_DOWN_VOLTAGE).withEnableFOC(true);
        this.pushdownController = new TorqueCurrentFOC(Settings.Intake.Pivot.PUSHDOWN_CURRENT.getAsDouble());
        this.rollerController = new DutyCycleOut(0.0).withEnableFOC(true);
        this.rollerFollower = new Follower(rollerMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed);

        rollerMotorRight.setControl(rollerFollower);

        this.pivotStalling = () -> pivotMotor.getStatorCurrent().getValue().gt(Settings.Intake.Pivot.STALL_CURRENT);
        this.leftRollerStalling = () -> rollerMotorLeft.getStatorCurrent().getValue().gt(Settings.Intake.Roller.STALL_CURRENT);
        this.rightRollerStalling = () -> rollerMotorRight.getStatorCurrent().getValue().gt(Settings.Intake.Roller.STALL_CURRENT);

        this.leftRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), DebounceType.kBoth);
        this.rightRollerDebouncer = new Debouncer(Settings.Intake.Roller.STALL_DEBOUNCE_SEC.in(Seconds), DebounceType.kBoth);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.isLimitSwitchHit = !pivotLimitSwitch.get();
        inputs.relativePosition = pivotMotor.getPosition().getValue();
        inputs.rollerVelocity = rollerMotorLeft.getVelocity().getValue();

        inputs.isPivotStalling = pivotStalling.getAsBoolean();
        inputs.isLeftRollerStalling = leftRollerDebouncer.calculate(leftRollerStalling.getAsBoolean());
        inputs.isRightRollerStalling = rightRollerDebouncer.calculate(rightRollerStalling.getAsBoolean());

        inputs.isApplyingPushdown = pivotMotor.getAppliedControl() == pushdownController;
    }

    @Override
    public void seedPivotAngle(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    @Override
    public void applyPushdown() {
        pivotMotor.setControl(pushdownController);
    }

    @Override
    public void applyHoming() {
        pivotMotor.setControl(homingController);
    }

    @Override
    public void applyPosition(Angle angle, int gainSlot) {
        pivotMotor.setControl(positionController.withPosition(angle).withSlot(gainSlot));
    }

    @Override
    public void applyRollerDutyCycle(double dutyCycle) {
        rollerMotorLeft.setControl(rollerController.withOutput(dutyCycle));
    }

    @Override
    public void stopRollerMotors() {
        rollerMotorLeft.stopMotor();
        rollerMotorRight.stopMotor();
        // re-add the follow control after stopMotor removes it
        rollerMotorRight.setControl(rollerFollower);
    }

    @Override
    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    @Override
    public void setVoltageOverride(Voltage voltage) {
        pivotMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void logHardwareSignals() {
        pivotSignals.logAll();
        rollerSignals.logAll();
    }
}
