/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToAllianceZone;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToHub;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.path.PathPlannerPath;

public class DepotOneCycleBottom extends SequentialCommandGroup {

    public DepotOneCycleBottom(PathPlannerPath... paths) {

        addCommands(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                .alongWith(new IntakeSetIntake()),
            new WaitCommand(2), // wait for intaking

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            
            new SwerveDriveAlignedToHub(),
            new FeederForward(),
            new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new IntakeAgitateOnce().repeatedly()),
            new FeederIdle(),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2])
                .alongWith(new IntakeSetIntake()),
            new SwerveDriveAlignedToAllianceZone(),
            new FeederForward(),
            new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new IntakeAgitateOnce().repeatedly()),
            new FeederIdle()
        );
    }
}