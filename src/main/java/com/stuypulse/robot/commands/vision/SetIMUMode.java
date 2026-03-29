package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIMUMode extends InstantCommand{
    private final LimelightVision vision;
    private final int mode;

    public SetIMUMode(int mode) {
        this.vision = LimelightVision.getInstance();
        this.mode = mode;
    }

    @Override
    public void initialize() {
        vision.setIMUMode(mode);
    }
}
