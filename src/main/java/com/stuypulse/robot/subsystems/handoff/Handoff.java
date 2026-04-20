package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Handoff extends SubsystemBase {
    
    private static final Handoff instance;
    private HandoffState state;

    static {
        instance = new HandoffImpl();
    }

    // static {
    //     if (Robot.isReal()){
    //         instance = new HandoffImpl();
    //     } else {
    //         instance = new HandoffSim();
    //     }
    // }

    public Handoff getInstance(){
        return instance;
    }

    public enum HandoffState {
        FORWARD(Settings.Handoff.FORWARD),
        BACKWARD(Settings.Handoff.BACKWARD),
        IDLE(Settings.Handoff.IDLE_DUTY);

        private double handoffMotorDutyCycle;

        private HandoffState(double handoffMotorDutyCycle){
            this.handoffMotorDutyCycle = handoffMotorDutyCycle;
        }

        public double getHandoffMotorDutyCycle(){
            return handoffMotorDutyCycle;
        }

    }

    public void setState(HandoffState state) {
        this.state = state;
    }

    public HandoffState getState(){
        return state;
    }

    protected Handoff(){
        setState(HandoffState.IDLE);
    }

    public void logMotor(TalonFX motor) {
        String stem = "Handoff/Motor/";
        
        SmartDashboard.putNumber(stem + "MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(stem + "SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(stem + "StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber(stem + "DutyCycle", motor.getDutyCycle().getValueAsDouble());
    }

    protected abstract void stopMotors();

    @Override
    public void periodic() {
        super.periodic();
        HandoffState currentState = getState();

        SmartDashboard.putNumber("Shooter/Handoff Target Duty Cycle", currentState.getHandoffMotorDutyCycle());

        SmartDashboard.putString("Handoff/State", currentState.name());
        SmartDashboard.putString("States/Handoff", currentState.name());
    }
}
