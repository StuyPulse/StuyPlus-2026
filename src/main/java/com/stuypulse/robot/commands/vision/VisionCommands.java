package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.subsystems.vision.LimelightVision.MegaTagMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class VisionCommands {
    private static final LimelightVision vision;

    static {
        vision = LimelightVision.getInstance();
    }

    public static Command setImuMode(int mode) {
        return Commands.runOnce(() -> vision.setIMUMode(mode)).ignoringDisable(true);
    }

    public static Command setMegaTagMode(MegaTagMode mode) {
        return Commands.runOnce(() -> vision.setMegaTagMode(mode)).ignoringDisable(true);
    }

    public static Command setPipeline(int pipelineIndex) {
        return Commands.runOnce(() -> vision.setPipeline(pipelineIndex)).ignoringDisable(true);
    }

    public static Command enable() {
        return Commands.runOnce(() -> vision.enable()).ignoringDisable(true);
    }
    
    public static Command disable() {
        return Commands.runOnce(() -> vision.disable()).ignoringDisable(true);
    }

    public static Command whitelistAllTags() {
        return Commands.runOnce(() -> vision.setTagWhitelist(Field.ALL_TAGS)).ignoringDisable(true);
    }
}
