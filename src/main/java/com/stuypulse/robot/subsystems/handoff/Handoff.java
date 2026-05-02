package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Handoff extends SubsystemBase {// handoff is feeder ---> shooter btw
    private static final Handoff instance;
    private HandoffState state;

    static {
        if (Robot.isReal()) {
            instance = new HandoffImpl();
        } else {
            instance = new HandoffSim();
        }
    }

    public static Handoff getInstance() {
        return instance;
    }

    protected Handoff() {
        this.state = HandoffState.IDLE;
    }

    public void setState(HandoffState state) {
        this.state = state;
    }

    public HandoffState getState() {
        return this.state;
    }

    public enum HandoffState {
        IDLE(Settings.Handoff.IDLE_DUTY_CYCLE),
        FORWARD(Settings.Handoff.FORWARD_DUTY_CYCLE),
        REVERSE(Settings.Handoff.REVERSE_DUTY_CYCLE);

        private double handoffMotorDutyCycle;

        private HandoffState(double handoffMotorDutyCycle) {
            this.handoffMotorDutyCycle = handoffMotorDutyCycle;
        }

        public double getHandoffDutyCycle() {
            return handoffMotorDutyCycle;
        }
    }

    protected abstract void stopMotors();

    @Override
    public void periodic() {
        final HandoffState currentState = getState();

        SmartDashboard.putString("Handoff/State", currentState.name());
        SmartDashboard.putString("States/Handoff", currentState.name());
        SmartDashboard.putNumber("Shooter/Handoff Target Duty Cycle", currentState.getHandoffDutyCycle());
    }
}