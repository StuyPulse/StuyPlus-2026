package com.stuypulse.robot.util.simulation;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TalonFXSimulation {
    private final TalonFX motor;
    private final DCMotorSim simMotor;

    public TalonFXSimulation(TalonFX motor, DCMotorSim simMotor) {
        this.motor = motor;
        this.simMotor = simMotor;
    }

    public TalonFXSimulation(TalonFX motor, DCMotorSim simMotor, TalonFXConfig config) {
        this.motor = motor;
        this.simMotor = simMotor;
        this.configure(config);
    }

    public void configure(TalonFXConfig config) {
        config.configure(motor);
    }

    public void setControl(ControlRequest request) {
        motor.setControl(request);
    }

    public TalonFX getMotor() {
        return motor;
    }

    public DCMotorSim getSimMotor() {
        return simMotor;
    }

    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(simMotor.getAngularVelocityRadPerSec());
    }

    public void update(double dtSeconds) {
        final TalonFXSimState state = motor.getSimState();

        simMotor.setInputVoltage(state.getMotorVoltage());
        simMotor.update(dtSeconds);

        final Angle rotorPositionRotations = Rotations.of(simMotor.getAngularPositionRad());
        final AngularVelocity feederRotorRPS = RPM.of(simMotor.getAngularVelocityRPM());

        state.setRawRotorPosition(rotorPositionRotations);
        state.setRotorVelocity(feederRotorRPS);
    }
}