package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class FeederIOSim implements FeederIO {
    private TalonFXSimulation feederMotor;
    private VoltageOut controller;
    private FeederState state;
    private SystemSim<DCMotorSim> feederSim;

    public FeederIOSim(int feederMotorID, double gearRatio) {
        setState(FeederState.IDLE);

        this.feederSim = SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(2),
                        Settings.Feeder.J.in(KilogramSquareMeters),
                        gearRatio),
                DCMotor.getKrakenX60(2))
        );

        this.feederMotor = new TalonFXSimulation(feederMotorID, gearRatio, this.feederSim);
        Motors.Feeder.LEADER_CONFIG.configure(feederMotor);
        
        this.controller = new VoltageOut(getState().getTargetVoltage()).withEnableFOC(true);
    }

    @Override
    public void stopMotors() {
        feederMotor.stopMotor();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.targetVoltage = getState().getTargetVoltage();
        inputs.currentAngularVelocity = feederMotor.getVelocity().getValue();
        
        this.feederSim.update(Settings.DT);
        this.feederMotor.refresh();
    }

    @Override
    public void setTargetVoltage(Voltage voltage) {
        feederMotor.setControl(controller.withOutput(voltage));
    }

    @Override
    public void setState(FeederState state) {
        this.state = state;
    }

    @Override
    public FeederState getState() {
        return this.state;
    }
}
