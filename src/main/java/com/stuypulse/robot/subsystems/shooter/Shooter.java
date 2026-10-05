/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.shooter.InterpolationCalculator;

import edu.wpi.first.units.measure.AngularVelocity;

public class Shooter extends FullSubsystem {
    private static final Shooter instance;

    static {
        switch (Settings.CURRENT_MODE) {
            case REAL -> instance = new Shooter(new ShooterIOTalonFX());

            case SIM -> instance = new Shooter(new ShooterIOSim());

            default -> instance = new Shooter(new ShooterIO() {});
        }
    }

    public static Shooter getInstance() {
        return instance;
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;
    private final ShooterIOOutputs outputs;

    @AutoLogOutput(key = "States/Shooter")
    private ShooterState state;

    private AngularVelocity bonusVelocity;

    private Shooter(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
        this.outputs = new ShooterIOOutputs();
        this.state = ShooterState.SHOOT;

        outputs.gainSlot = 0;

        bonusVelocity = RPM.zero();
    }

    public ShooterState getState() {
        return this.state;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public void setGainSlot(int slot) {
        outputs.gainSlot = slot;
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
        MANUAL_HUB(ShooterConstants.ShooterSettings.MANUAL_HUB_RPM);

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

    public AngularVelocity getCurrentAngularVelocity() {
        return inputs.shooterMotorRightInputs.velocity;
    }

    @AutoLogOutput(key = "Shooter/isSpunUp")
    public boolean shooterSpunUp() {
        return getCurrentAngularVelocity().gte(getState().getTargetAngularVelocity().minus(ShooterConstants.ShooterSettings.SHOOTER_SPUN_UP_TOLERANCE));
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

        if (!Settings.EnabledSubsystems.SHOOTER.get()) {
            stopMotors();
            return;
        } 

        runVelocity(state.getTargetAngularVelocity().plus(bonusVelocity));
    }
    
    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    private void runVelocity(AngularVelocity velocity) {
        outputs.mode = ShooterIO.ShooterIOOutputMode.VELOCITY;
        outputs.targetVelocity = velocity;
    }

    private void stopMotors() {
        outputs.mode = ShooterIO.ShooterIOOutputMode.STOP;
    }
}
