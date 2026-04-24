package com.stuypulse.robot.subsystems.feeder;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSim extends Feeder {
    private final TalonFXSimulation feederLeader;
    private final DCMotorSim feederSim;
    private final DutyCycleOut feederController;

    public FeederSim() {
        feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2),
                Settings.Feeder.J.in(KilogramSquareMeters),
                Settings.Feeder.GEAR_RATIO
            ),
            DCMotor.getKrakenX60(2)
        );

        feederLeader = new TalonFXSimulation(feederSim).configure(Motors.Feeder.LEADER_CONFIG);

        feederController = new DutyCycleOut(0)
            .withEnableFOC(true);
    }

    @Override
    public AngularVelocity getCurrentAngularVelocity() {
        return feederLeader.getMotor().getVelocity().getValue();
    }

    @Override
    protected void stopMotors() {
        feederLeader.stopMotor();
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            stopMotors();
            return;
        }

        feederLeader.setControl(feederController.withOutput(getState().getTargetDutyCycle())); // apply control to leader before grabbing state to update other motors

        feederLeader.update(Settings.DT);

        SmartDashboard.putNumber("Feeder/Leader Current", feederLeader.getMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Leader Voltage", feederLeader.getMotor().getMotorVoltage().getValueAsDouble());

        RobotVisualizer.getInstance().updateFeeder(getCurrentAngularVelocity());

        super.periodic();
    }
}