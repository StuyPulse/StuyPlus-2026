package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.stuypulse.robot.constants.Motors;

import com.stuypulse.robot.util.LoggedSignals;
import com.stuypulse.robot.util.LoggedSignals.SignalLocation;

import edu.wpi.first.units.measure.Voltage;

public class FeederIOImpl implements FeederIO {
    private TalonFX feederMotor;
    private VoltageOut controller;
    private LoggedSignals signals;
    private FeederState state;

    public FeederIOImpl(int feederMotorID, CANBus canBus) {
        setState(FeederState.IDLE);

        this.feederMotor = new TalonFX(feederMotorID, canBus);
        Motors.Feeder.LEADER_CONFIG.configure(feederMotor);
        
        this.controller = new VoltageOut(getState().getTargetVoltage()).withEnableFOC(true);

        this.signals = new LoggedSignals(SignalLocation.CANIVORE, "Feeder/",
            feederMotor.getSupplyCurrent(),
            feederMotor.getStatorCurrent(),
            feederMotor.getVelocity());
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.currentAngularVelocity = feederMotor.getVelocity().getValue();
        inputs.targetVoltage = getState().getTargetVoltage();
    }

    @Override
    public void stopMotors() {
        feederMotor.stopMotor();
    }

    @Override
    public void setTargetVoltage(Voltage voltage) {
        feederMotor.setControl(controller.withOutput(voltage));
    }

    @Override
    public void setState(FeederState state) {
        this.state = state;
    }

    @Override
    public FeederState getState() {
        return this.state;
    }

    @Override
    public void log(FeederIOInputs inputs) {
        FeederIO.super.log(inputs);
        this.signals.logAll();
    }
}
