package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Handoff extends SubsystemBase {
    private final static Handoff instance;
    private HandoffState state;

    static {
        if (Robot.isReal()) {
            instance = new ShooterImpl();
        } else {
            //instance = new ShooterSim();
        }
    }

    public static Handoff getInstance() {
        return instance;
    }

    protected Handoff() {
        setState(HandoffState.IDLE);
    }

    public void setState(Handoff state) {
        this.state = state;
    }

    public HandoffState getState() {
        return this.state;
    }

    public enum HandoffState {
        IDLE(Settings.Handoff.IDLE_DUTY),
        STOP(Settings.Handoff.STOP_DUTY),
        TOHAND(Settings.Handoff.TOHAND_DUTY);
        //NOTTOHAND();

        private double handoffMotorDutyCycle;

        private HandoffState(double handoffMotorDutyCycle) {
            this.handoffMotorDutyCycle = handoffMotorDutyCycle;
        }

        public double getHandoffMotorDutyCycle() {
            return handoffMotorDutyCycle;
        }
    }

    public abstract AngularVelocity getCurrentAngularVelocity();
    protected abstract void stopMotors();

    @Override
    public void periodic() {
        final HandoffState currentState = getState();

        SmartDashboard.putNumber("Handoff/Handoff Target Duty Cycle", currentState.getHandoffMotorDutyCycle());

        SmartDashboard.putString("Handoff/State", currentState.name());
        SmartDashboard.putString("States/Handoff", currentState.name());
    }
}