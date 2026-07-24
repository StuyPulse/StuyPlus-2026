package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

public class FeederIOSim implements FeederIO {
    private final TalonFXSimulation feederMotor;
    private final VoltageOut controller;
    private final SystemSim<DCMotorSim> feederSim;

    public FeederIOSim(int feederMotorID, double gearRatio) {
        this.feederSim = SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        Settings.Feeder.J.in(KilogramSquareMeters),
                        gearRatio),
                DCMotor.getKrakenX60(1))
        );

        this.feederMotor = new TalonFXSimulation(feederMotorID, gearRatio, this.feederSim);
        Motors.Feeder.LEADER_CONFIG.configure(feederMotor);
        
        this.controller = new VoltageOut(0).withEnableFOC(true);
    }

    @Override
    public void stopMotors() {
        feederMotor.stopMotor();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.voltage = feederMotor.getMotorVoltage().getValue();
        inputs.angularVelocity = feederMotor.getVelocity().getValue();
        inputs.position = feederMotor.getPosition().getValue();
    }

    @Override
    public void setTargetVoltage(Voltage voltage) {
        feederMotor.setControl(controller.withOutput(voltage));
    }

    @Override
    public void updateSim() {
        this.feederSim.update(Settings.DT);
        this.feederMotor.refresh();
    }
}
