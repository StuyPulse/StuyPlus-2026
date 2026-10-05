package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public class ShooterIOReal extends ShooterIOBase {
    public ShooterIOReal() {
        super(new LoggedTalonFX(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_RIGHT),
            new LoggedTalonFX(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_CENTER),
            new LoggedTalonFX(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_LEFT));
    }
}
