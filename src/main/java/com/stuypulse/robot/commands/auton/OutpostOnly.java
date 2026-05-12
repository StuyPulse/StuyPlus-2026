package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.intake.IntakeAgitateWhileOuttaking;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.commands.vision.SetVisionDisabled;
import com.stuypulse.robot.commands.vision.SetVisionEnabled;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OutpostOnly extends SequentialCommandGroup {

    public OutpostOnly(PathPlannerPath... paths) {
        addCommands(new SetVisionDisabled(), new SwerveResetPose(paths[0].getStartingHolonomicPose().get()), CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]), // new WaitCommand(20).deadlineFor(new IntakeAgitateWhileOuttaking()),
        new SetVisionEnabled());
    }
}
