package com.stuypulse.robot.subsystems.handoff;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.LoggedSignals;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import edu.wpi.first.units.measure.Voltage;

public class HandoffIOImpl implements HandoffIO {
    private final TalonFX handoffMotor;
    private final VoltageOut handoffController;
    private final LoggedSignals signals;

    private final BooleanSupplier handoffStalling;
    private final Debouncer stallDebouncer;

    public HandoffIOImpl(int handoffMotorId, CANBus canbus) {
        this.handoffMotor = new TalonFX(handoffMotorId, canbus);
        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);

        this.handoffController = new VoltageOut(0).withEnableFOC(true);
        this.signals = new LoggedSignals(LoggedSignals.SignalLocation.CANIVORE, "Handoff/",
            handoffMotor.getSupplyCurrent(),
            handoffMotor.getStatorCurrent(),
            handoffMotor.getVelocity());
        
        this.handoffStalling = () -> Math.abs(handoffMotor.getStatorCurrent().getValueAsDouble()) > Settings.Handoff.STALL_CURRENT;
        this.stallDebouncer = new Debouncer(Settings.Handoff.STALL_DEBOUNCE, DebounceType.kRising);
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        inputs.angularVelocity = handoffMotor.getVelocity().getValue();
        inputs.voltage = handoffMotor.getMotorVoltage().getValue();
        inputs.isStalling = stallDebouncer.calculate(this.handoffStalling.getAsBoolean());
    }

    @Override
    public void stopMotors() {
        handoffMotor.stopMotor();
    }

    @Override
    public void setMotorVoltage(Voltage voltage) {
        this.handoffMotor.setControl(this.handoffController.withOutput(voltage));
    }

    @Override
    public void logHardwareSignals() {
        this.signals.logAll();
    }
}
