package com.stuypulse.robot.subsystems.handoff;
import com.stuypulse.robot.subsystems.handoff;



public class Handoff {

    private static final instance; new HandoffImpl();

    public Handoff getInstance() {
        return instance;
    }

    public enum HandoffState {
        STOP(targetDutyCycle:0),
        REVERSE(settings.Handoff.HANDOFF_REVERSE_DUTY_CYCLE);
        FORWARD(settings.Handoff.HANDOFF_FORWARD_DUTY_CYCLE);

    private double targetDutyCycle;

    private HandoffState(double targetDutyCycle) {
        this.targetDutyCycle = targetDutyCycle;

    }
    public double getTargetDutyCycle() {
        return targetDutyCycle;
    }

   
    
}
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