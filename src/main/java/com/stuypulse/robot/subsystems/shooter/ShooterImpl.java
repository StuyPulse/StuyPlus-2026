package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports.ShooterPorts;
import com.stuypulse.robot.constants.Motors;

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


    }
}
