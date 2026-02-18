package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    private static final Intake instance;
    private IntakeState state;
    protected TrapezoidProfile.State currentPivotState;
    protected TrapezoidProfile.State targetPivotState;

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
        IDLE(Settings.Intake.IDLE_ANGLE, Settings.Intake.IDLE_VOLTAGE), // (rollers do not run)
        INTAKE(Settings.Intake.INTAKE_ANGLE, Settings.Intake.INTAKE_VOLTAGE), // (sucks in the balls) [pivot down,
                                                                              // rollers running]
        OUTTAKE(Settings.Intake.OUTTAKE_ANGLE, Settings.Intake.OUTTAKE_VOLTAGE), // (trips the balls out) [pivot down,
                                                                                 // rollers running reverse]
        UP(Settings.Intake.AGITATE_ANGLE, 0),
        DOWN(-Settings.Intake.AGITATE_ANGLE, 0);

        private double voltage;
        private double angle;

        private IntakeState(double angle, double voltage) {
            this.voltage = voltage;
            this.angle = angle;
        }

        protected double getAngle() {
            return angle;
        }

        protected double getVoltage() {
            return voltage;
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

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/State", getState().name());
        SmartDashboard.putString("States/Intake", getState().name());
    }
}   