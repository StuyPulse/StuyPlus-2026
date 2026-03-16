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
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.path.PathPlannerPath;

public class OutpostDepotOnePointFiveCycle extends SequentialCommandGroup {

    public OutpostDepotOnePointFiveCycle(PathPlannerPath... paths){
        addCommands(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                .alongWith(new IntakeSetIntake()),
                new WaitCommand(1), // Wait at outpost for balls

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new WaitCommand(2), //Intake at depot

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new SwerveDriveAlignedToHub(),
                new WaitCommand(Settings.Shooter.SHOOT_TIME_AUTO).deadlineFor(new FeederForward(), new IntakeAgitateOnce().repeatedly()),
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])   
                .alongWith(new FeederIdle(), new IntakeSetIntake()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
            
            new SwerveDriveAlignedToAllianceZone(),
            new WaitCommand(2).deadlineFor(new FeederForward(), new IntakeAgitateOnce().repeatedly()), // Ferry from neutral zone

            new FeederIdle().alongWith(new IntakeSetIntake())
        );
    }
}