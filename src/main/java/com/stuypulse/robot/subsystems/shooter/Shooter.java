package com.stuypulse.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.shooter.InterpolationCalculator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
        setState(ShooterState.SHOOT);
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

        SmartDashboard.putNumber(stem + "RPM", motor.getVelocity().getValue().in(RPM));
    }

    public enum ShooterState {

        SOTM(() -> 0.0, Settings.Shooter.SOTM_DUTY), // TODO: Make actual suppliers
        FOTM(() -> 0.0, Settings.Shooter.FOTM_DUTY),
        IDLE(() -> 0.0, Settings.Shooter.IDLE_DUTY),
        SHOOT(() -> InterpolationCalculator.interpolateShotInfo().targetRPM(), Settings.Shooter.SHOOT_DUTY),
        FERRY(() -> InterpolationCalculator.interpolateFerryingInfo().targetRPM(), Settings.Shooter.FERRY_DUTY),
        MANUAL_HUB(() -> Settings.Shooter.MANUAL_HUB_RPM.in(RPM), 0);

        private DoubleSupplier RPMSupplier;

        private ShooterState(DoubleSupplier RPMSupplier, double handoffMotorDutyCycle) {
            this.RPMSupplier = RPMSupplier;
        }

        public AngularVelocity getTargetAngularVelocity() {
            return RPM.of(RPMSupplier.getAsDouble());
        }

    }

    public abstract AngularVelocity getCurrentAngularVelocity();
    protected abstract void stopMotors();

    public abstract SysIdRoutine getShooterSysIdRoutine();
    public abstract void setVoltageOverride(Voltage voltage);

    @Override
    public void periodic() {
        final ShooterState currentState = getState();

        SmartDashboard.putNumber("Shooter/Top Target RPM", currentState.getTargetAngularVelocity().in(RPM));
        SmartDashboard.putString("Shooter/State", currentState.name());
        SmartDashboard.putString("States/Shooter", currentState.name());
    }
}

