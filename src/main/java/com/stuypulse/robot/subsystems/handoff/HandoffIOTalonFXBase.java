package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public abstract class HandoffIOTalonFXBase implements HandoffIO {
    private final LoggedTalonFX handoffMotor;

    private final VoltageOut handoffController;

    public HandoffIOTalonFXBase(LoggedTalonFX motor) {
        handoffMotor = motor;
        handoffController = new VoltageOut(0).withEnableFOC(true);

        HandoffConstants.HandoffMotorConfigs.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        handoffMotor.updateInputs(inputs.handoffMotorInputs);
    }

    @Override
    public void applyOutputs(HandoffIOOutputs outputs) {
        switch (outputs.mode) {
            case VOLTAGE -> handoffMotor.setControl(handoffController.withOutput(outputs.voltage));

            case STOP -> handoffMotor.stopMotor();
        }
    }   
}
