package com.stuypulse.robot.util.simulation;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TalonFXSimulation {
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

    private static int id = 50;

    private final TalonFX motor;
    private final SystemSim simMotor;

    private TalonFXSimulation(SystemSim adapter) {
        this.motor = new TalonFX(getID());
        this.simMotor = adapter;
    }

    public TalonFXSimulation(DCMotorSim sim) {
        this(SystemSim.of(sim));
    }

    public TalonFXSimulation(FlywheelSim sim) {
        this(SystemSim.of(sim));
    }

    public TalonFXSimulation(ElevatorSim sim) {
        this(SystemSim.of(sim));
    }

    public TalonFXSimulation(SingleJointedArmSim sim) {
        this(SystemSim.of(sim));
    }

    public static int getID() {
        id += 1;
        return id;
    }

    public TalonFXSimulation configure(TalonFXConfig config) {
        config.configure(motor);
        return this;
    }

    public void setControl(ControlRequest request) {
        motor.setControl(request);
    }

    public TalonFX getMotor() {
        return motor;
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public void update(double dtSeconds) {
        final TalonFXSimState state = motor.getSimState();

        simMotor.setInputVoltage(Volts.of(state.getMotorVoltage()));
        simMotor.update(dtSeconds);

        state.setRawRotorPosition(simMotor.getAngularPosition().in(Rotations));
        state.setRotorVelocity(simMotor.getAngularVelocity().in(RotationsPerSecond));
    }

    public void update(Time dtSeconds) {
        this.update(dtSeconds.in(Seconds));
    }
}
