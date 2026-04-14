package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederImpl extends Feeder {
    private final TalonFX feederMotor1;
    private final TalonFX feederMotor2;
    private final DutyCycleOut controller;

    public FeederImpl() {
        feederMotor1 = new TalonFX(Ports.Feeder.FEEDER_MOTOR_1);
        feederMotor2 = new TalonFX(Ports.Feeder.FEEDER_MOTOR_2);

        Motors.Feeder.MOTOR_CONFIG_1.configure(feederMotor1);
        Motors.Feeder.MOTOR_CONFIG_2.configure(feederMotor2);
        
        controller = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);

        feederMotor2.setControl(new Follower(Ports.Feeder.FEEDER_MOTOR_1, MotorAlignmentValue.Opposed)); //TODO: figure out motor alignment
    }

    @Override
    public void periodic() {
        if (EnabledSubsystems.FEEDER.get()) {
            super.periodic();
            feederMotor1.setControl(controller.withOutput(getState().getTargetDutyCycle()));
        
            SmartDashboard.putNumber("Feeder/Target Duty Cycle", getState().getTargetDutyCycle());
            SmartDashboard.putNumber("Feeder/Current RPM", feederMotor1.getVelocity().getValueAsDouble() * 60);
        }
    }
}