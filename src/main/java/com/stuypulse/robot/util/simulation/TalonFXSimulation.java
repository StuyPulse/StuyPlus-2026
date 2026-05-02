/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.simulation;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class TalonFXSimulation extends TalonFX {
    private interface SystemSim {
        public void setInputVoltage(Voltage voltage);

        public void update(double dtSeconds);

        public Angle getAngularPosition();

        public AngularVelocity getAngularVelocity();

        public static SystemSim of(DCMotorSim sim) {
            return new SystemSim() {
                @Override
                public void setInputVoltage(Voltage voltage) {
                    sim.setInputVoltage(voltage.in(Volts));
                }
                @Override
                public void update(double dt) {
                    sim.update(dt);
                }
                @Override
                public Angle getAngularPosition() {
                    return Radians.of(sim.getAngularPositionRad());
                }
                @Override
                public AngularVelocity getAngularVelocity() {
                    return RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
                }
            };
        }

        public static SystemSim of(FlywheelSim sim) {
            return new SystemSim() {
                @Override
                public void setInputVoltage(Voltage voltage) {
                    sim.setInputVoltage(voltage.in(Volts));
                }
                @Override
                public void update(double dt) {
                    sim.update(dt);
                }
                @Override
                public Angle getAngularPosition() {
                    return Radians.of(0); // flywheels don't track position
                }
                @Override
                public AngularVelocity getAngularVelocity() {
                    return RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
                }
            };
        }

        public static SystemSim of(ElevatorSim sim) {
            return new SystemSim() {
                @Override
                public void setInputVoltage(Voltage voltage) {
                    sim.setInputVoltage(voltage.in(Volts));
                }
                @Override
                public void update(double dt) {
                    sim.update(dt);
                }
                @Override
                public Angle getAngularPosition() {
                    return Radians.of(sim.getPositionMeters());
                }
                @Override
                public AngularVelocity getAngularVelocity() {
                    return RadiansPerSecond.of(sim.getVelocityMetersPerSecond());
                }
            };
        }

        public static SystemSim of(SingleJointedArmSim sim) {
            return new SystemSim() {
                @Override
                public void setInputVoltage(Voltage voltage) {
                    sim.setInputVoltage(voltage.in(Volts));
                }
                @Override
                public void update(double dt) {
                    sim.update(dt);
                }
                @Override
                public Angle getAngularPosition() {
                    return Radians.of(sim.getAngleRads());
                }
                @Override
                public AngularVelocity getAngularVelocity() {
                    return RadiansPerSecond.of(sim.getVelocityRadPerSec());
                }
            };
        }
    }

    private final SystemSim simMotor;

    private TalonFXSimulation(int port, SystemSim adapter) {
        super(port);
        this.simMotor = adapter;
    }

    public TalonFXSimulation(int port, DCMotorSim sim) {
        this(port, SystemSim.of(sim));
    }

    public TalonFXSimulation(int port, FlywheelSim sim) {
        this(port, SystemSim.of(sim));
    }

    public TalonFXSimulation(int port, ElevatorSim sim) {
        this(port, SystemSim.of(sim));
    }

    public TalonFXSimulation(int port, SingleJointedArmSim sim) {
        this(port, SystemSim.of(sim));
    }

    public void configure(TalonFXConfig config) {
        config.configure(this);
    }

    public void update(double dtSeconds) {
        final TalonFXSimState state = this.getSimState();

        simMotor.setInputVoltage(Volts.of(state.getMotorVoltage()));
        simMotor.update(dtSeconds);

        state.setRawRotorPosition(simMotor.getAngularPosition().in(Rotations));
        state.setRotorVelocity(simMotor.getAngularVelocity().in(RotationsPerSecond));
    }

    public void update(Time dtSeconds) {
        this.update(dtSeconds.in(Seconds));
    }
}
