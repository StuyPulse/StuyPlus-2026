package com.stuypulse.robot.commands.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.subsystems.led.LEDController;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
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
        
    }

    private boolean isShooting() {
        return shooter.getState() == ShooterState.SHOOTING;
    }
    
    private boolean isFerrying() {
        return shooter.getState() == ShooterState.FERRYING;
    }

    private boolean feederForward() {
        return feeder.getState() == FeederState.FORWARD;
    }

    private boolean feederReversed() {
        return feeder.getState() == FeederState.REVERSE;

    }

    private boolean intaking(){
        return intake.getState() == IntakeState.INTAKE;
    }

    @Override
    public void execute() {
        
        if (ShiftTimer.getInstance().getRobotMode() == RobotState.DISABLED) { 
            leds.applyPattern(Settings.LED.DISABLED);
        } else if (isShooting()) {
            leds.applyPattern(Settings.LED.shooterShooting);
        } 
        else if (feederForward()) {
            leds.applyPattern(Settings.LED.feederForward);
        }
        else if (feederReversed()) {
            leds.applyPattern(Settings.LED.feederReverse);
        }
        else if (isFerrying()) {
            leds.applyPattern(Settings.LED.ferrying);
        }
        else if (intaking()){
            leds.applyPattern(Settings.LED.intaking);
        }
        else {
            leds.applyPattern(LEDPattern.kOff);
        }
        
    }
}
