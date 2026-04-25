package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDDefaultCommand extends Command{
    private final LEDController leds;

    private final Shooter shooter;
    private final Feeder feeder;
    private final Intake intake;
    private final Handoff handoff;

    public LEDDefaultCommand() {
        leds = LEDController.getInstance();

        shooter = Shooter.getInstance();
        feeder = Feeder.getInstance();
        intake = Intake.getInstance();
        handoff = Handoff.getInstance();

        addRequirements(leds);
    }

    @Override
    public void execute() {
        if (!Settings.EnabledSubsystems.LED.get()) {
            return;
        }

        if (DriverStation.isDisabled()) {
            leds.applyPattern(Settings.LED.DISABLED);
            return;
        }

        //These probably won't actually be what we want the LEDs to be showing
        //TODO: Figure out what we want the LEDs to show

        switch (shooter.getState()) {
            case SHOOT -> leds.applyShoot(Settings.LED.SHOOTING);
            case FERRY -> leds.applyShoot(Settings.LED.FERRYING);
            case MANUAL_HUB -> leds.applyShoot(Settings.LED.MANUAL);
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
            case HOMING_DOWN -> leds.applyIntake(Settings.LED.HOMING_DOWN);
            default -> leds.applyIntake(LEDPattern.kOff);
        }

        switch (handoff.getState()) {
            case FORWARD -> leds.applyHandoff(Settings.LED.HANDOFF_FORWARD);
            case REVERSE -> leds.applyHandoff(Settings.LED.HANDOFF_REVERSE);
            case IDLE -> leds.applyHandoff(LEDPattern.kOff);
        }

    }
}
