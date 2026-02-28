package com.stuypulse.robot.commands.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.commands.shooter.ShooterSetState;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.feeder.Feeder.FeederState;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;
import com.stuypulse.robot.subsystems.shifttimer.ShiftTimer;
import com.stuypulse.robot.subsystems.shifttimer.ShiftTimer.RobotState;
import com.stuypulse.robot.subsystems.vision.LimelightVision;


public class LEDDefaultCommand extends Command {
    private final LEDController leds;
    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;

    public LEDDefaultCommand() {
        this.leds = LEDController.getInstance();
        this.shooter = Shooter.getInstance();
        this.feeder = Feeder.getInstance();
        this.intake = Intake.getInstance();

        addRequirements(leds);
    }

    // LED Docs: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
    
    @Override
    public void execute() {
        leds.applyPattern(
            ShiftTimer.getInstance().getRobotMode() == RobotState.DISABLED 
                ? Settings.LED.DISABLED
                : LEDPattern.kOff
        );

        switch (shooter.getState()) {
            case SHOOTING -> leds.applyShoot(Settings.LED.SHOOTING);
            case FERRYING -> leds.applyShoot(Settings.LED.FERRYING);
            case IDLE -> leds.applyShoot(LEDPattern.kOff);
        }

        switch (feeder.getState()) {
            case FORWARD -> leds.applyFeed(Settings.LED.FEEDER_FORWARD);
            case REVERSE -> leds.applyFeed(Settings.LED.FEEDER_REVERSE);
            case STOP -> leds.applyFeed(LEDPattern.kOff);
        }

        switch (intake.getState()) {
            case INTAKE -> leds.applyIntake(Settings.LED.INTAKING);
            case OUTTAKE -> leds.applyIntake(Settings.LED.OUTTAKING);
            default -> leds.applyIntake(LEDPattern.kOff);
        }
    }
}