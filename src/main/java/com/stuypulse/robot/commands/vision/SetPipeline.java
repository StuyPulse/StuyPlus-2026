package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.util.vision.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetPipeline extends InstantCommand {
    private int pipeline;
    public SetPipeline(int pipeline) {
        this.pipeline = pipeline;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", pipeline);
    }
}