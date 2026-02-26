package com.stuypulse.robot.commands.led;

import edu.wpi.first.wpilibj.LEDPattern;
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

    private boolean isShooting() {
        return shooter.getState() == ShooterState.SHOOTING;
    }
    
    private boolean isFerrying() {
        return shooter.getState() == ShooterState.FERRYING;
    }

    private boolean shooterIdle() {
        return shooter.getState() == ShooterState.IDLE;
    }

    private boolean feederForward() {
        return feeder.getState() == FeederState.FORWARD;
    }

    private boolean feederReversed() {
        return feeder.getState() == FeederState.REVERSE;
    }

    private boolean feederIdle() {
        return feeder.getState() == FeederState.STOP;
    }

    private boolean intaking(){
        return intake.getState() == IntakeState.INTAKE;
    }

    private boolean outtaking(){
        return intake.getState() == IntakeState.OUTTAKE;
    }

    private boolean isAgitating(){
        return intake.getState() == IntakeState.UP || intake.getState() == IntakeState.DOWN;
    }

    private boolean intakeIdle() {
        return intake.getState() == IntakeState.IDLE;
    }


// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
    
    @Override
    public void execute() {
        
        if (ShiftTimer.getInstance().getRobotMode() == RobotState.DISABLED) { 
            leds.applyPattern(Settings.LED.DISABLED);
        } else {
            leds.applyPattern(LEDPattern.kOff);
        }
        
        if (isShooting()) {
            leds.applyShoot(Settings.LED.SHOOTING);
        } 
        if (isFerrying()) {
            leds.applyShoot(Settings.LED.FERRYING);
        } 
        if (shooterIdle()) {
            leds.applyShoot(LEDPattern.kOff);
        }
        if (feederForward()) {
            leds.applyFeed(Settings.LED.FEEDER_FORWARD);
        } 
        if (feederReversed()) {
            leds.applyFeed(Settings.LED.FEEDER_REVERSE);
        } 
        if (feederIdle()) {
            leds.applyFeed(LEDPattern.kOff);
        }
        if (intaking()){
            leds.applyIntake(Settings.LED.INTAKING);
        } 
        if (outtaking()){
            leds.applyIntake(Settings.LED.OUTTAKING);
        } 
        if (isAgitating()){
            leds.applyIntake(Settings.LED.AGITATING);
        } 
        if (intakeIdle()) {
            leds.applyIntake(LEDPattern.kOff);
        }
    }
}