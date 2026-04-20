package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Handoff extends SubsystemBase {
    private static final Handoff instance;
    private static final AngularVelocityUnit RPM = null;

    static {
        instance = new HandoffImpl();
    }

    public Handoff getInstance() {
        return instance;
    }

    public enum HandoffState {
        STOP(0),
        REVERSE(Settings.Handoff.HANDOFF_REVERSE_DUTY_CYCLE),
        FORWARD(Settings.Handoff.HANDOFF_FORWARD_DUTY_CYCLE);

        private double targetDutyCycle;
        
        private HandoffState(double targetDutyCycle) {
            this.targetDutyCycle = targetDutyCycle;

        }
        public double getTargetDutyCycle() {
            return targetDutyCycle;
        }
    }
    private HandoffState state;

    protected Handoff() {
        this.state = HandoffState.STOP;
    }
     public void setState (HandoffState state) {
        this.state = state;
    }
    
    public HandoffState getState() {
        return state;
    }

    public abstract AngularVelocity getCurrentAngularVelocity();
    protected abstract void stopMotors();

    @Override
    public void periodic() {
        final HandoffState currentState = getState();

        // Logging
        SmartDashboard.putNumber("Handoff/Target Duty Cycle", currentState.getTargetDutyCycle());
        SmartDashboard.putNumber("Handoff/Current RPM", getCurrentAngularVelocity().in(RPM));

        SmartDashboard.putString("Handoff/State", currentState.name());
        SmartDashboard.putString("States/Handoff", currentState.name());
    }
}