package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetVisionEnabled extends InstantCommand{
    private LimelightVision vision;

    public SetVisionEnabled() {
        this.vision = LimelightVision.getInstance();
    }

    @Override
    public void initialize() {
        vision.enable();
    }
}
