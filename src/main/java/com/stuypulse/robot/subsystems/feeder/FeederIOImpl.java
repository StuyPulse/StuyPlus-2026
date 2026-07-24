package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.util.LoggedSignals;
import com.stuypulse.robot.util.LoggedSignals.SignalLocation;

import edu.wpi.first.units.measure.Voltage;

public class FeederIOImpl implements FeederIO {
    private final TalonFX feederMotor;
    private final VoltageOut controller;
    private final LoggedSignals signals;

    public FeederIOImpl(int feederMotorID, CANBus canBus) {

        this.feederMotor = new TalonFX(feederMotorID, canBus);
        Motors.Feeder.LEADER_CONFIG.configure(feederMotor);
        
        this.controller = new VoltageOut(0).withEnableFOC(true);

        this.signals = new LoggedSignals(SignalLocation.CANIVORE, "Feeder/",
            feederMotor.getSupplyCurrent(),
            feederMotor.getStatorCurrent(),
            feederMotor.getVelocity());
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.angularVelocity = feederMotor.getVelocity().getValue();
        inputs.voltage = feederMotor.getMotorVoltage().getValue();
    }

    @Override
    public void stopMotors() {
        feederMotor.stopMotor();
    }

    @Override
    public void setMotorVoltage(Voltage voltage) {
        feederMotor.setControl(controller.withOutput(voltage));
    }

    @Override
    public void logHardwareSignals() {
        this.signals.logAll();
    }
}
