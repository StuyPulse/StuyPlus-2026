package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Feeder extends SubsystemBase{
    private static final Feeder instance;
    private FeederState state;

    static {
        if (Robot.isReal()) {
            instance = new FeederImpl();
        } else {
            instance = new FeederSim();
        }
    }

    public static Feeder getInstance(){
        return instance;
    }

    public enum FeederState {
        STOP(0),
        REVERSE(Settings.Feeder.FEEDER_REVERSE_DUTY_CYCLE),
        FORWARD(Settings.Feeder.FEEDER_FORWARD_DUTY_CYCLE);

        private double targetDutyCycle;

        private FeederState(double targetDutyCycle){
            this.targetDutyCycle = targetDutyCycle;
        }

        public double getTargetDutyCycle(){
            return this.targetDutyCycle;
        }
    }
    
    protected Feeder() {
        this.state = FeederState.STOP;
    }

    public void setState(FeederState state){
        this.state = state;
    }

    public FeederState getState() {
        return state;
    }

    public abstract AngularVelocity getCurrentAngularVelocity();
    protected abstract void stopMotors();

    @Override
    public void periodic() {
        final FeederState currentState = getState();

        // Logging
        SmartDashboard.putNumber("Feeder/Target Duty Cycle", currentState.getTargetDutyCycle());
        SmartDashboard.putNumber("Feeder/Current Angular Velocity", getCurrentAngularVelocity().in(RPM));

        SmartDashboard.putString("Feeder/State", currentState.name());
        SmartDashboard.putString("States/Feeder", currentState.name());
    }
}