package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public abstract class FeederIOTalonFXBase implements FeederIO {
    private final TalonFX feederMotor;

    private final VoltageOut feederController;
    
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;

    protected FeederIOTalonFXBase(TalonFX feederMotor) {
        this.feederMotor = feederMotor;
        Motors.Feeder.LEADER_CONFIG.configure(feederMotor);
        feederController = new VoltageOut(0).withEnableFOC(true);

        position = feederMotor.getPosition();
        velocity = feederMotor.getVelocity();
        voltage = feederMotor.getMotorVoltage();
        supplyCurrent = feederMotor.getSupplyCurrent();
    }

    @Override
    public void stopMotors() {
        feederMotor.stopMotor();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.voltage = voltage.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
    }

    @Override
    public void setTargetVoltage(Voltage voltage) {
        feederMotor.setControl(feederController.withOutput(voltage));
    }
}
