package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX extends ShooterIOTalonFXBase {
    private static TalonFX getShooterMotor(int id) {
        return new TalonFX(id, Settings.CANBUS);
    }

    public ShooterIOTalonFX() {
        super(getShooterMotor(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_RIGHT),
            getShooterMotor(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_CENTER),
            getShooterMotor(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_LEFT));
    }
}
