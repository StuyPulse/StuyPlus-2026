package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class HandoffImpl extends Handoff {
    private final TalonFX handoffMotor;
    private final DutyCycleOut handoffController;

    public HandoffImpl(){
        handoffMotor = new TalonFX(Ports.ShooterPorts.HANDOFF_MOTOR, Settings.CANIVORE);
        handoffController = new DutyCycleOut(getState().getHandoffMotorDutyCycle()).withEnableFOC(true);
        Motors.Shooter.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);
    }

    @Override
    protected void stopMotors(){
        handoffMotor.stopMotor();
    }

    @Override
    public void periodic(){
        if(!Settings.EnabledSubsystems.HANDOFF.get()) {
            stopMotors();
            return;
        }

         CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
         Shooter shooter = Shooter.getInstance();

        if (!(swerve.isAlignedToTarget(Field.getHubPose())) && shooter.getState() == ShooterState.SHOOT) {
            setState(HandoffState.IDLE);
        }

         DutyCycleOut handoffControl = handoffController.withOutput(getState().getHandoffMotorDutyCycle());

        handoffMotor.setControl(handoffControl);
        this.logMotor(handoffMotor);
    }   
}

