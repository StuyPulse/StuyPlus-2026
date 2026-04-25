package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {
    private static final Shooter shooter;

    static {
        shooter = Shooter.getInstance();
    }

    public static Command setShoot() {
        return Commands.runOnce(() -> shooter.setState(ShooterState.SHOOT), shooter);
    }

    public static Command setFerry() {
        return Commands.runOnce(() -> shooter.setState(ShooterState.FERRY), shooter);
    }

    public static Command setIdle() {
        return Commands.runOnce(() -> shooter.setState(ShooterState.IDLE), shooter);
    }

    public static Command setManual() {
        return Commands.runOnce(() -> shooter.setState(ShooterState.MANUAL_HUB), shooter);
    }
}
