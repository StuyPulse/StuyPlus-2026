package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX extends ShooterIOTalonFXBase {
    public ShooterIOTalonFX() {
        super(new TalonFX(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_RIGHT),
            new TalonFX(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_CENTER),
            new TalonFX(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_LEFT));
    }
}
