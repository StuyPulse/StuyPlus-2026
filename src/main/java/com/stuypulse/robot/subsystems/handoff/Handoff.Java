package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Handoff extends SubsystemBase {
    private final static Handoff instance;
    private HandoffState state;

    static {
        instance = new HandoffImpl();
    }

    public static Handoff getInstance() {
        return instance;
    }

    protected Handoff() {
        setState(HandoffState.IDLE);
    }

    public void setState(HandoffState state) {
        this.state = state;
    }

    public HandoffState getState() {
        return this.state;
    }

    public void logMotor(String motorName, TalonFX motor) {
        String stem = "Handoff/Motors/" + motorName + "/";
        
        SmartDashboard.putNumber(stem + "MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(stem + "SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(stem + "StatorCurrent", motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber(stem + "DutyCycle", motor.getDutyCycle().getValueAsDouble());
    }

    public enum HandoffState {
        IDLE(Settings.Handoff.IDLE_DUTY),
        STOP(Settings.Handoff.STOP_DUTY),
        REVERSE(Settings.Handoff.REVERSE_DUTY),
        FORWARD(Settings.Handoff.FORWARD_DUTY);

        private double handoffMotorDutyCycle;

        private HandoffState(double handoffMotorDutyCycle) {
            this.handoffMotorDutyCycle = handoffMotorDutyCycle;
        }

        public double getHandoffMotorDutyCycle() {
            return handoffMotorDutyCycle;
        }
    }

    protected abstract void stopMotors();

    @Override
    public void periodic() {
        final HandoffState currentState = getState();

        SmartDashboard.putNumber("Handoff/Handoff Target Duty Cycle", currentState.getHandoffMotorDutyCycle());

        SmartDashboard.putString("Handoff/State", currentState.name());
        SmartDashboard.putString("States/Handoff", currentState.name());
    }
}
