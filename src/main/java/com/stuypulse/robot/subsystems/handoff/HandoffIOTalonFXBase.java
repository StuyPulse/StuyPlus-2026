package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

public abstract class HandoffIOTalonFXBase implements HandoffIO {
    private final TalonFX handoffMotor;

    private final VoltageOut handoffController;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;

    public HandoffIOTalonFXBase(TalonFX motor) {
        handoffMotor = motor;
        handoffController = new VoltageOut(0).withEnableFOC(true);

        HandoffConstants.HandoffMotorConfigs.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);

        position = handoffMotor.getPosition();
        velocity = handoffMotor.getVelocity();
        voltage = handoffMotor.getMotorVoltage();
        supplyCurrent = handoffMotor.getSupplyCurrent();
        statorCurrent = handoffMotor.getStatorCurrent();
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        inputs.position = position.refresh().getValue();
        inputs.velocity = velocity.refresh().getValue();
        inputs.voltage = voltage.refresh().getValue();
        inputs.supplyCurrent = supplyCurrent.refresh().getValue();
        inputs.statorCurrent = statorCurrent.refresh().getValue();
    }

    @Override
    public void applyOutputs(HandoffIOOutputs outputs) {
        switch (outputs.mode) {
            case VOLTAGE -> handoffMotor.setControl(handoffController.withOutput(outputs.voltage));

            case STOP -> handoffMotor.stopMotor();
        }
    }   
}
