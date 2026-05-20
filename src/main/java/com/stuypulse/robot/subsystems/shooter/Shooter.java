/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.shooter.InterpolationCalculator;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

public abstract class Shooter extends SubsystemBase {

    private static final Shooter instance;

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
        DogLog.log(stem + "MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
        DogLog.log(stem + "SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        DogLog.log(stem + "StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        DogLog.log(stem + "RPM", motor.getVelocity().getValue().in(RPM));
    }

    public enum ShooterState {

        // SOTM(() -> 0.0), 
        // FOTM(() -> 0.0),
        IDLE(() -> 0.0),
        SHOOT(Settings.Shooter.SHOOT_TUNING_RPM), //TODO:Replace with interpolated RPM after data is gathered
        FERRY(Settings.Shooter.FERRY_TUNING_RPM),
        // SHOOT(() -> InterpolationCalculator.interpolateShotInfo().targetRPM()),
        // FERRY(() -> InterpolationCalculator.interpolateFerryingInfo().targetRPM()),
        MANUAL_HUB(() -> Settings.Shooter.MANUAL_HUB_RPM.in(RPM));

        private DoubleSupplier RPMSupplier;

        private ShooterState(DoubleSupplier RPMSupplier) {
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

    public boolean shooterSpunUp() {
        return getCurrentAngularVelocity().gte(Settings.Shooter.MIN_SHOOTER_VELOCITY);
    }

    @Override
    public void periodic() {
        final ShooterState currentState = getState();
        DogLog.log("Shooter/Target RPM", currentState.getTargetAngularVelocity().in(RPM));
        DogLog.log("Shooter/State", currentState.name());
        DogLog.log("States/Shooter", currentState.name());
    }
}
