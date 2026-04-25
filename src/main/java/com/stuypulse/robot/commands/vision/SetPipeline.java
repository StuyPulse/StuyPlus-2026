package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.vision.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetPipeline extends InstantCommand {
    private final LimelightVision vision;
    private int pipeline;
    public SetPipeline(int pipeline) {
        this.pipeline = pipeline;
        this.vision = LimelightVision.getInstance();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        vision.setPipeline(pipeline);
    }
}