/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.shooter.InterpolationCalculator;
import com.stuypulse.robot.util.simulation.RobotVisualizer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    private static final Shooter instance;

    static {
        if (Robot.isReal()) {
            instance = new Shooter(new ShooterIOTalonFX());
        } else {
            instance = new Shooter(new ShooterIOSim());
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;
    @AutoLogOutput(key = "States/Shooter")
    private ShooterState state;

    private AngularVelocity bonusVelocity;

    private Shooter(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
        this.state = ShooterState.SHOOT;

        io.setGainsSlot(0);

        bonusVelocity = RPM.zero();
    }

    public ShooterState getState() {
        return this.state;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public void setGainSlot(int slot) {
        io.setGainsSlot(slot);
    }

    /** Enum representing the different possible states of the shooter. */
    public enum ShooterState {

        // SOTM(() -> 0.0), 
        // FOTM(() -> 0.0),
        /** Shooter doesn't run. */
        IDLE(() -> 0.0),
        /** Shooter wheels spin at it's target RPM, interpolated based on distance to hub. */
        // SHOOT(Settings.Shooter.SHOOT_TUNING_RPM), //TODO:Replace with interpolated RPM after data is gathered
        /** Shooter wheels spin at it's target RPM, interpolated based on distance to ferry zone. */
        // FERRY(Settings.Shooter.FERRY_TUNING_RPM),
        SHOOT(() -> InterpolationCalculator.interpolateShotInfo().targetRPM()),
        FERRY(() -> InterpolationCalculator.interpolateFerryingInfo().targetRPM()),
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

    private Optional<Voltage> voltageOverride;

    //getters
    public void setVoltageOverride(Voltage voltage) {
        this.voltageOverride = Optional.of(voltage);
    }

    public AngularVelocity getCurrentAngularVelocity() {
        return inputs.velocity;
    }

    @AutoLogOutput(key = "Shooter/isSpunUp")
    public boolean shooterSpunUp() {
        return getCurrentAngularVelocity().gte(getState().getTargetAngularVelocity().minus(Settings.Shooter.SHOOTER_SPUN_UP_TOLERANCE));
    }

    public SysIdRoutine getShooterSysIdRoutine() {
        return SysId.getRoutine(
                Settings.Shooter.RAMP_RATE,
                Settings.Shooter.STEP_VOLTAGE,
                "Shooter",
                this::setVoltageOverride,
                () -> inputs.position,
                () -> inputs.velocity,
                () -> inputs.voltage,
                getInstance());
    }

    //setters
    public void addToBonusVelocity(double velocity) {
        bonusVelocity = bonusVelocity.plus(RPM.of(velocity));
    }

    public void resetBonusVelocity() {
        bonusVelocity = RPM.of(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        final ShooterState currentState = getState();

        if (!Settings.EnabledSubsystems.SHOOTER.get()) {
            io.stopMotors();
        } else if (voltageOverride.isPresent()) {
            io.setTargetVoltage(voltageOverride.get());
        } else {
            io.setTargetVelocity(currentState.getTargetAngularVelocity().plus(bonusVelocity));
        }
        RobotVisualizer.getInstance().updateShooter(inputs.velocity);
    }
}
