/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

    /** Enum representing the different possible states of the shooter. */
    public enum ShooterState {

        // SOTM(() -> 0.0), 
        // FOTM(() -> 0.0),
        /** Shooter doesn't run. */
        IDLE(() -> 0.0),
        /** Shooter wheels spin at it's target RPM, interpolated based on distance to hub. */
        SHOOT(Settings.Shooter.SHOOT_TUNING_RPM), //TODO:Replace with interpolated RPM after data is gathered
        /** Shooter wheels spin at it's target RPM, interpolated based on distance to ferry zone. */
        FERRY(Settings.Shooter.FERRY_TUNING_RPM),
        // SHOOT(() -> InterpolationCalculator.interpolateShotInfo().targetRPM()),
        // FERRY(() -> InterpolationCalculator.interpolateFerryingInfo().targetRPM()),
        /** Shooter wheels spin at a predetermined constant rate without interpolation. */
        MANUAL_HUB(Settings.Shooter.MANUAL_HUB_RPM);

        /** The supplier for the target RPM of the shooter in the corresponding state. */
        private DoubleSupplier RPMSupplier;

        /**
         * Constructs a ShooterState with the given supplier for the target RPM of the shooter.
         * @param RPMSupplier the supplier for the target RPM of the shooter in the corresponding state
         */
        private ShooterState(DoubleSupplier RPMSupplier) {
            this.RPMSupplier = RPMSupplier;
        }

        /**
         * Gets the target angular velocity of the shooter in the corresponding state by converting the target RPM from the supplier to an AngularVelocity.
         * @return the target angular velocity of the shooter
         */
        public AngularVelocity getTargetAngularVelocity() {
            return RPM.of(RPMSupplier.getAsDouble());
        }
    }

    public abstract AngularVelocity getCurrentAngularVelocity();

    protected abstract void stopMotors();

    public abstract SysIdRoutine getShooterSysIdRoutine();

    public abstract void setVoltageOverride(Voltage voltage);

    public boolean shooterSpunUp() {
        return getCurrentAngularVelocity().gte(getState().getTargetAngularVelocity().minus(RPM.of(100)));
    }

    @Override
    public void periodic() {
        final ShooterState currentState = getState();
        DogLog.log("Shooter/Target RPM", currentState.getTargetAngularVelocity().in(RPM));
        DogLog.log("Shooter/State", currentState.name());
        DogLog.forceNt.log("States/Shooter", currentState.name());
        DogLog.forceNt.log("Shooter/At Target RPM", shooterSpunUp());
    }
}
