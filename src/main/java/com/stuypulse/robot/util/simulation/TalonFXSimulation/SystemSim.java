package com.stuypulse.robot.util.simulation.TalonFXSimulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 *
 *
 * <h2>System Simulation</h2>
 *
 * <p>This interface defines the contract for a system simulation, which can be used to abstract the
 * behavior of all WPILib linear system sims in one interface.
 *
 * @see TalonFXSimulation
 * @author Faizaan (https://github.com/Faizaan-J)
 */
public interface SystemSim<T> {
    /**
     * Sets the input voltage for the simulation.
     *
     * @param voltage The input voltage to set for the simulation.
     */
    void setInputVoltage(Voltage voltage);

    /**
     * Updates the simulation
     *
     * @param dt The time step to advance the simulation
     */
    void update(Time dt);

    /**
     * Gets the current position of the mechanism in the simulation.
     *
     * @return The current position of the mechanism.
     */
    Angle getMechanismPosition();

    /**
     * Gets the current velocity of the mechanism in the simulation.
     *
     * @return The current velocity of the mechanism.
     */
    AngularVelocity getMechanismVelocity();

    /**
     * Gets the wrapped WPILib linear system simulation instance.
     *
     * @return The wrapped WPILib linear system simulation instance.
     */
    T getLinearSystemSim();

    static SystemSim<DCMotorSim> of(DCMotorSim dcMotorSim) {
        return new SystemSim<DCMotorSim>() {
            @Override
            public void setInputVoltage(Voltage voltage) {
                dcMotorSim.setInputVoltage(voltage.in(Volts));
            }

            @Override
            public void update(Time dt) {
                dcMotorSim.update(dt.in(Seconds));
            }

            @Override
            public Angle getMechanismPosition() {
                return dcMotorSim.getAngularPosition();
            }

            @Override
            public AngularVelocity getMechanismVelocity() {
                return dcMotorSim.getAngularVelocity();
            }

            @Override
            public DCMotorSim getLinearSystemSim() {
                return dcMotorSim;
            }
        };
    }

    static SystemSim<FlywheelSim> of(FlywheelSim flywheelSim) {
        return new SystemSim<FlywheelSim>() {
            private Angle position = Rotations.of(0);

            @Override
            public void setInputVoltage(Voltage voltage) {
                flywheelSim.setInputVoltage(voltage.in(Volts));
            }

            @Override
            public void update(Time dt) {
                flywheelSim.update(dt.in(Seconds));
                position = position.plus(flywheelSim.getAngularVelocity().times(dt));
            }

            @Override
            public Angle getMechanismPosition() {
                return position;
            }

            @Override
            public AngularVelocity getMechanismVelocity() {
                return flywheelSim.getAngularVelocity();
            }

            @Override
            public FlywheelSim getLinearSystemSim() {
                return flywheelSim;
            }
        };
    }

    static SystemSim<SingleJointedArmSim> of(SingleJointedArmSim armSim) {
        return new SystemSim<SingleJointedArmSim>() {
            @Override
            public void setInputVoltage(Voltage voltage) {
                armSim.setInputVoltage(voltage.in(Volts));
            }

            @Override
            public void update(Time dt) {
                armSim.update(dt.in(Seconds));
            }

            @Override
            public Angle getMechanismPosition() {
                return Radians.of(armSim.getAngleRads());
            }

            @Override
            public AngularVelocity getMechanismVelocity() {
                return RadiansPerSecond.of(armSim.getVelocityRadPerSec());
            }

            @Override
            public SingleJointedArmSim getLinearSystemSim() {
                return armSim;
            }
        };
    }

    static SystemSim<ElevatorSim> of(ElevatorSim elevatorSim, Distance drumRadius) {
        return new SystemSim<ElevatorSim>() {
            @Override
            public void setInputVoltage(Voltage voltage) {
                elevatorSim.setInputVoltage(voltage.in(Volts));
            }

            @Override
            public void update(Time dt) {
                elevatorSim.update(dt.in(Seconds));
            }

            @Override
            public Angle getMechanismPosition() {
                return Radians.of(elevatorSim.getPositionMeters() / drumRadius.in(Meters));
            }

            @Override
            public AngularVelocity getMechanismVelocity() {
                return RadiansPerSecond.of(
                        elevatorSim.getVelocityMetersPerSecond() / drumRadius.in(Meters));
            }

            @Override
            public ElevatorSim getLinearSystemSim() {
                return elevatorSim;
            }
        };
    }
}