package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WhitelistAllTags extends InstantCommand{
    private final LimelightVision vision;

    public WhitelistAllTags() {
        this.vision = LimelightVision.getInstance();
    }

    @Override
    public void initialize() {
        vision.setTagWhitelist(Field.ALL_TAGS);
    }
}
