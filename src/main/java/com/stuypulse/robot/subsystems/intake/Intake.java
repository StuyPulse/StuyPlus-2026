package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    private static final Intake instance;
    private IntakeState state;

    static {
        if (Robot.isReal()){
            instance = new IntakeImpl();
        }
         else {
            instance = new IntakeSim();
        }
    }
    
    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState {
        IDLE(Settings.Intake.IDLE_ANGLE, Settings.Intake.IDLE_DUTY_CYCLE), // (rollers do not run)
        INTAKE(Settings.Intake.INTAKE_ANGLE, Settings.Intake.INTAKE_DUTY_CYCLE), // (sucks in the balls) [pivot down,
                                                                              // rollers running]
        OUTTAKE(Settings.Intake.OUTTAKE_ANGLE, Settings.Intake.OUTTAKE_DUTY_CYCLE), // (trips the balls out) [pivot down,
                                                                                 // rollers running reverse]
        UP(Settings.Intake.AGITATE_UP_ANGLE, Settings.Intake.IDLE_DUTY_CYCLE),
        DOWN(Settings.Intake.AGITATE_DOWN_ANGLE, Settings.Intake.IDLE_DUTY_CYCLE);

        private double dutyCycle;
        private Rotation2d angle;

        private IntakeState(Rotation2d angle, double dutyCycle) {
            this.dutyCycle = dutyCycle;
            this.angle = angle;
        }

        protected Rotation2d getTargetAngle() {
            return angle;
        }

        protected double getTargetDutyCycle() {
            return dutyCycle;
        }
    }

    protected Intake() {
        this.state = IntakeState.IDLE;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }

    public abstract Rotation2d getRelativePosition();
    public abstract boolean atAngle();

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/State", getState().name());
        SmartDashboard.putString("States/Intake", getState().name());
    }
}   