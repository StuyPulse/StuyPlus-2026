package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Settings;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Handoff extends SubsystemBase{
    private static final Handoff instance;
    private HandoffState state;

    static {
        instance = new HandoffImpl(); //change later when sim-ing
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
        String stem = "Shooter/Motors/" + motorName + "/";
        
        SmartDashboard.putNumber(stem + "MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(stem + "SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(stem + "StatorCurrent", motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber(stem + "RPM", motor.getVelocity().getValue().in(RPM));
    }
    public enum HandoffState {
        IDLE(Settings.Handoff.IDLE_DUTYCYCLE),
        FORWARD(Settings.Handoff.FORWARD_DUTYCYCLE),
        REVERSE(Settings.Handoff.REVERSE_DUTYCYCLE);

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
    public void periodic(){
        final HandoffState currentState = getState();
        
        SmartDashboard.putNumber("Shooter/Handoff Target Duty Cycle", currentState.getHandoffMotorDutyCycle());
    }
}