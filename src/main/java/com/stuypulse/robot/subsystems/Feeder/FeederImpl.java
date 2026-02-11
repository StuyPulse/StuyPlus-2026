package com.stuypulse.robot.subsystems.Feeder;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederImpl extends Feeder {
    private final TalonFX feederMotor1;
    private final TalonFX feederMotor2;

    public FeederImpl() {
        feederMotor1 = new TalonFX(Ports.Feeder.FEEDER_MOTOR_1);
        feederMotor2 = new TalonFX(Ports.Feeder.FEEDER_MOTOR_2);

        Motors.Feeder.MOTOR_CONFIG_1.configure(feederMotor1);
        Motors.Feeder.MOTOR_CONFIG_2.configure(feederMotor2);

        feederMotor2.setControl(new Follower(Ports.Feeder.FEEDER_MOTOR_1, MotorAlignmentValue.Opposed)); //TODO: figure out motor alignment
    }

    public void setMotors() {
        feederMotor1.setControl(new VelocityVoltage(getState().getTargetRPM()));
    }

    @Override
    public void periodic() {
        super.periodic();
        setMotors();

        SmartDashboard.putNumber("Feeder/Target RPM", getState().getTargetRPM());
        SmartDashboard.putNumber("Feeder/Current RPM", feederMotor1.getVelocity().getValueAsDouble());
    }
}