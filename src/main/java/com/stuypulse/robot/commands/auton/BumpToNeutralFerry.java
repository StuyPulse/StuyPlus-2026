package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.feeder.FeederForward;
import com.stuypulse.robot.commands.feeder.FeederIdle;
import com.stuypulse.robot.commands.intake.IntakeAgitateOnce;
import com.stuypulse.robot.commands.intake.IntakeSetIdle;
import com.stuypulse.robot.commands.intake.IntakeSetIntake;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveAlignedToAllianceZone;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class BumpToNeutralFerry extends SequentialCommandGroup {

    public BumpToNeutralFerry(PathPlannerPath... paths) {
        addCommands(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
                .alongWith(new IntakeSetIntake()),

            new SwerveDriveAlignedToAllianceZone(),
            new WaitCommand(5).deadlineFor(new IntakeAgitateOnce().repeatedly(), new FeederForward()),
            new WaitCommand(5).deadlineFor(new IntakeSetIntake(), new FeederIdle()),
            new WaitCommand(5).deadlineFor(new IntakeAgitateOnce().repeatedly(), new FeederForward()),
            new IntakeSetIntake().alongWith(new FeederIdle())
        );
    }
}