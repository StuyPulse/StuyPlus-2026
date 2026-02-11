package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Ports.ShooterPorts;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.util.ShooterInterpolation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.util.FerryInterpolation;

public class ShooterImpl extends Shooter {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX shooterMotor3;
    private final TalonFX bottomMotor1;
    private final TalonFX bottomMotor2;

    public ShooterImpl() {
        shooterMotor1 = new TalonFX(ShooterPorts.SHOOTER_MOTOR_1);
        shooterMotor2 = new TalonFX(ShooterPorts.SHOOTER_MOTOR_2);
        shooterMotor3 = new TalonFX(ShooterPorts.SHOOTER_MOTOR_3);

        bottomMotor1 = new TalonFX(ShooterPorts.BOTTOM_MOTOR_1);
        bottomMotor2 = new TalonFX(ShooterPorts.BOTTOM_MOTOR_2);

        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor1);
        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor2);
        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor3);
        Motors.Shooter.MOTOR_CONFIG.configure(bottomMotor1);
        Motors.Shooter.MOTOR_CONFIG.configure(bottomMotor2);

        // TODO: find out which motor has to be inverted
        shooterMotor2.setControl(new Follower(ShooterPorts.SHOOTER_MOTOR_1, MotorAlignmentValue.Aligned));
        shooterMotor3.setControl(new Follower(ShooterPorts.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));

        bottomMotor2.setControl(new Follower(ShooterPorts.BOTTOM_MOTOR_2, MotorAlignmentValue.Opposed));
    }

    public double getShootSpeed(){
        return ShooterInterpolation.getRPM(5 /*TODO: we do  */);
    }

    public double getFerrySpeed(){
        return FerryInterpolation.getRPM(5); // TODO: write interpolation for this 六七 shi
    }

    public void setVoltagesBasedOnState() {
        double targetRPM = getState().getShooterSpeed();

        
        shooterMotor1.setControl(new VelocityVoltage(targetRPM / 60)); // TODO: periodic stuff
        shooterMotor2.setControl(new VelocityVoltage(targetRPM / 60));
        shooterMotor3.setControl(new VelocityVoltage(targetRPM / 60));
        
        bottomMotor1.setControl(new VelocityVoltage(getState().getBottomMotorSpeed()));
        bottomMotor2.setControl(new VelocityVoltage(getState().getBottomMotorSpeed()));
    }

    @Override
    public void periodic() {
        super.periodic();

        setVoltagesBasedOnState();

        SmartDashboard.putNumber("Shooter/Target RPM", getState().getShooterSpeed());
        SmartDashboard.putNumber("Shooter/Current RPM", shooterMotor1.getVelocity().getValueAsDouble() * 60);
    }
}
