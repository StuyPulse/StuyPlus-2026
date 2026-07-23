package com.stuypulse.robot.util.simulation.TalonFXSimulation;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public interface SystemSim<T> {
    void setInputVoltage(Voltage voltage);
    void update(Time dt);
    Angle getMechanismPositionRotations();
    AngularVelocity getMechanismVelocity();
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
            public Angle getMechanismPositionRotations() {
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
            public Angle getMechanismPositionRotations() {
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
            public Angle getMechanismPositionRotations() {
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
            public Angle getMechanismPositionRotations() {
                return Radians.of(elevatorSim.getPositionMeters() / drumRadius.in(Meters));
            }

            @Override
            public AngularVelocity getMechanismVelocity() {
                return RadiansPerSecond.of(elevatorSim.getVelocityMetersPerSecond() / drumRadius.in(Meters));
            }

            @Override
            public ElevatorSim getLinearSystemSim() {
                return elevatorSim;
            }
        };
    }
}
