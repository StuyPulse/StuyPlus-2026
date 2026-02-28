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

// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
    
    @Override
    public void execute() {
        
        if (ShiftTimer.getInstance().getRobotMode() == RobotState.DISABLED) { 
            leds.applyPattern(Settings.LED.DISABLED);
        } else {
            leds.applyPattern(LEDPattern.kOff);
        }

        switch (shooter.getState()) {
            case SHOOTING:
                leds.applyShoot(Settings.LED.shooterShooting);
            break;
            case FERRYING:
                leds.applyShoot(Settings.LED.ferrying);
            break;
            case IDLE:
                leds.applyShoot(LEDPattern.kOff);
            break; 
        }
        switch (feeder.getState()) {
            case FORWARD:
                leds.applyFeed(Settings.LED.feederForward);
            break;
            case REVERSE:
                leds.applyFeed(Settings.LED.feederReverse);
            break;
            case STOP:
                leds.applyFeed(LEDPattern.kOff);
            break;
        }
        switch (intake.getState()) {
            case INTAKE:
                leds.applyIntake(Settings.LED.intaking);
            break;
            case OUTTAKE:
                leds.applyIntake(Settings.LED.outtaking);
            break;
            case IDLE:
                leds.applyIntake(LEDPattern.kOff);
            break;
        }
    }
}
