package com.stuypulse.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.shooter.InterpolationCalculator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    private final static Shooter instance;
    private ShooterState state;

    static {
        if (Robot.isReal()) {
            instance = new ShooterImpl();
        } else {
            instance = new ShooterSim();
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    protected Shooter() {
        setState(ShooterState.IDLE);
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return this.state;
    }

    public void logMotor(String motorName, TalonFX motor) {
        String stem = "Shooter/Motors/" + motorName + "/";
        
        SmartDashboard.putNumber(stem + "MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber(stem + "SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(stem + "StatorCurrent", motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber(stem + "RPM", motor.getVelocity().getValueAsDouble());
    }

    public enum ShooterState {

        SOTM(() -> 0.0, Settings.Shooter.SOTM_DUTY), // TODO: Make actual suppliers
        FOTM(() -> 0.0, Settings.Shooter.FOTM_DUTY),
        IDLE(() -> 0.0, Settings.Shooter.IDLE_DUTY),
        SHOOT(() -> InterpolationCalculator.interpolateShotInfo().targetRPM(), Settings.Shooter.SHOOT_DUTY),
        FERRY(() -> InterpolationCalculator.interpolateFerryingInfo().targetRPM(), Settings.Shooter.FERRY_DUTY);

        private DoubleSupplier RPMSupplier;
        private double handoffMotorDutyCycle;

        private ShooterState(DoubleSupplier RPMSupplier, double handoffMotorDutyCycle) {
            this.RPMSupplier = RPMSupplier;
            this.handoffMotorDutyCycle = handoffMotorDutyCycle;
        }

        public double getRPM() {
            return RPMSupplier.getAsDouble();
        }

        public double getHandoffMotorDutyCycle() {
            return handoffMotorDutyCycle;
        }
    }

    public abstract double getCurrentRPM();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Top Target RPM", getState().getRPM());
        SmartDashboard.putNumber("Shooter/Handoff Target Duty Cycle", getState().getHandoffMotorDutyCycle());

        SmartDashboard.putString("Shooter/State", getState().name());
        SmartDashboard.putString("States/Shooter", getState().name());
    }
}

