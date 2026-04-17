package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
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
        Camera[] limelights = Cameras.LimelightCameras;

        for  (int i = 0; i < limelights.length; i++) {
            Camera currentLimelight = limelights[i];
            if (currentLimelight.isEnabled()) {
                LimelightHelpers.setPipelineIndex(currentLimelight.getName(), pipeline);
            }
        }
    }
}