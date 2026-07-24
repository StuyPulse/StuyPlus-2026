package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.VoltageOut;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.util.simulation.TalonFXSimIds;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HandoffIOSim implements HandoffIO {
    private final SystemSim<DCMotorSim> handoffSim;
    private final TalonFXSimulation handoffMotor;
    private final VoltageOut handoffController;

    public HandoffIOSim() {
        this.handoffSim = SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        Settings.Handoff.J_KG_METERS_SQUARED,
                        Settings.Handoff.GEAR_RATIO),
                DCMotor.getKrakenX60(1))
        );

        this.handoffMotor = new TalonFXSimulation(TalonFXSimIds.get("Handoff/Motor"), Settings.Handoff.GEAR_RATIO, this.handoffSim);
        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);

        this.handoffController = new VoltageOut(0).withEnableFOC(true);
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        inputs.angularVelocity = handoffMotor.getVelocity().getValue();
        inputs.voltage = handoffMotor.getMotorVoltage().getValue();
        inputs.isStalling = false; // sim never stalls
    }

    @Override
    public void stopMotors() {
        handoffMotor.stopMotor();
    }

    @Override
    public void setMotorVoltage(Voltage voltage) {
        this.handoffMotor.setControl(this.handoffController.withOutput(voltage));
    }

    @Override
    public void updateSim() {
        this.handoffSim.update(Settings.DT);
        this.handoffMotor.refresh();
    }
}
