package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public abstract class FeederIOTalonFXBase implements FeederIO {
    private final LoggedTalonFX feederMotor;

    private final VoltageOut feederController;
    
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;

    protected FeederIOTalonFXBase(LoggedTalonFX feederMotor) {
        this.feederMotor = feederMotor;
        FeederConstants.FeederMotorConfigs.LEADER_CONFIG.configure(feederMotor);
        feederController = new VoltageOut(0).withEnableFOC(true);

        position = feederMotor.getPosition();
        velocity = feederMotor.getVelocity();
        voltage = feederMotor.getMotorVoltage();
        supplyCurrent = feederMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederMotor.updateInputs(inputs.feederMotorInputs);
    }

    @Override
    public void applyOutputs(FeederIOOutputs outputs) {
        switch (outputs.mode) {
            case VOLTAGE -> feederMotor.setControl(feederController.withOutput(outputs.voltage));

            case STOP -> feederMotor.stopMotor();
        }
    }
}
