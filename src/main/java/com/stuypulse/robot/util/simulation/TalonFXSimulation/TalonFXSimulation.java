/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.simulation.TalonFXSimulation;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;

/**
 *
 *
 * <h2>TalonFX Simulation</h2>
 *
 * <p>This is a simulation wrapper for the TalonFX motor controller. It allows for simulating the
 * behavior of a CTRE motor controller by using WPILib's linear system simulation classes. This
 * class is designed to be used in a conjunction with a SystemSim instance that abstracts all types
 * of WPILib linear sims.
 *
 * @see SystemSim
 * @author Faizaan (https://github.com/Faizaan-J)
 */
public class TalonFXSimulation extends LoggedTalonFX {
    private final SystemSim<?> simMotor;
    private double gearRatio;

    /**
     * Creates a new TalonFXSimulation instance.
     *
     * @param port The port of the motor controller (TalonFX).
     * @param gearRatio The gear ratio of the motor.
     * @param adapter The simulation adapter for the motor.
     */
    public TalonFXSimulation(int port, double gearRatio, SystemSim<?> adapter) {
        super(port);
        this.gearRatio = gearRatio;
        this.simMotor = adapter;
    }

    /**
     * Configures the motor with the given configuration.
     *
     * @param config The configuration to apply to the motor.
     */
    public void configure(TalonFXConfig config) {
        config.configure(this);
    }

    /**
     * Call once AFTER configuring the motors to link the orientation of this motor to a reference
     * motor. This ensures that the gear ratio is correctly oriented for the simulation.
     *`
     * @param reference The reference motor to compensate for orientation. This should be the
     *     "leader" motor usually.
     */
    public void linkToReference(TalonFXSimulation reference) {
        final MotorOutputConfigs thisConfigs = new MotorOutputConfigs();
        this.getConfigurator().refresh(thisConfigs);

        final MotorOutputConfigs referenceConfigs = new MotorOutputConfigs();
        reference.getConfigurator().refresh(referenceConfigs);

        final boolean isSameOrientation = thisConfigs.Inverted == referenceConfigs.Inverted;
        this.gearRatio = Math.abs(this.gearRatio) * (isSameOrientation ? 1 : -1);
    }

    /**
     * Refeshes the simulation state of the motor. This should be called only AFTER updating the
     * corresponding SystemSim.
     */
    public void refresh() {
        final TalonFXSimState simState = this.getSimState();

        this.simMotor.setInputVoltage(
                simState.getMotorVoltageMeasure().times(Math.signum(gearRatio)));

        Angle rotorPosition = simMotor.getMechanismPosition().times(this.gearRatio);
        AngularVelocity rotorVelocity = this.simMotor.getMechanismVelocity().times(this.gearRatio);

        simState.setRawRotorPosition(rotorPosition);
        simState.setRotorVelocity(rotorVelocity);
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }
}