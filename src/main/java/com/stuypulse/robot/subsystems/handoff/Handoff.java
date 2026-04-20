package com.stuypulse.robot.subsystems.handoff;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

public abstract class Handoff extends SubsystemBase {// handoff is feeder ---> shooter btw
    private static final Handoff instance;
    private HandoffState state;

    static {
        instance = new HandoffImpl();
    }

    public static Handoff getInstance() {
        return instance;
    }

    public void logMotor(String motorName, TalonFX motor) {
        String stem = "Handoff/Motors/" + motorName + "/";

        SmartDashboard.putNumber(stem + "MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(stem + "SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(stem + "StatorCurrent", motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber(stem + "DutyCycle", motor.getDutyCycle().getValueAsDouble());
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
        ACTIVE(Settings.Handoff.ACTIVE_DUTY_CYCLE),
        REVERSE(Settings.Handoff.REVERSE_DUTY_CYCLE);

        public double handoffMotorDutyCycle;

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
        SmartDashboard.putNumber("Shooter/Handoff Target Duty Cycle", currentState.getHandoffMotorDutyCycle());
    }
}