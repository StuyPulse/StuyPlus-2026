package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public abstract class FeederIOBase implements FeederIO {
    private final LoggedTalonFX feederMotor;

    private final VoltageOut feederController;

    protected FeederIOBase(LoggedTalonFX feederMotor) {
        this.feederMotor = feederMotor;
        FeederConstants.FeederMotorConfigs.LEADER_CONFIG.configure(feederMotor);
        feederController = new VoltageOut(0).withEnableFOC(true);
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
