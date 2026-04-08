package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetVisionDisabled extends InstantCommand{
    private final LimelightVision vision;

    public SetVisionDisabled() {
        this.vision = LimelightVision.getInstance();
    }

    @Override
    public void initialize() {
        vision.disable();
    }
}
