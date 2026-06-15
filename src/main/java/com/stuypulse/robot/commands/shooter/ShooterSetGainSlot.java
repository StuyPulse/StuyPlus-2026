package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetGainSlot extends InstantCommand{
    private final Shooter shooter;
    private final int slot;

    public ShooterSetGainSlot(int slot) {
        this.shooter = Shooter.getInstance();
        this.slot = slot;
    }

    @Override
    public void initialize() {
        shooter.setGainSlot(slot);
    }
}
