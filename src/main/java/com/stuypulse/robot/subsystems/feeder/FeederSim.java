package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;
import com.stuypulse.robot.constants.Motors;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSim extends Feeder {
    private final TalonFXSimulation feederLeader;
    private final TalonFXSimulation feederFollower;
    private final DCMotorSim feederSim;
    private final DutyCycleOut feederController;

    public FeederSim() {
        feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2),
                Settings.Feeder.J_KG_METERS_SQUARED,
                Settings.Feeder.GEAR_RATIO
            ),
            DCMotor.getKrakenX60(2)
        );

        feederLeader = new TalonFXSimulation(feederSim).configure(Motors.Feeder.LEADER_CONFIG);
        feederFollower = new TalonFXSimulation(feederSim).configure(Motors.Feeder.FOLLOWER_CONFIG);

        feederFollower.setControl(new Follower(feederLeader.getMotor().getDeviceID(), MotorAlignmentValue.Opposed));

        feederController = new DutyCycleOut(0)
            .withEnableFOC(true);
    }

    @Override
    public double getCurrentRPM() {
        return feederLeader.getMotor().getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            feederLeader.setControl(feederController.withOutput(0));
            return;
        }

        feederLeader.setControl(feederController.withOutput(getState().getTargetDutyCycle())); // apply control to leader before grabbing state to update other motors

        feederLeader.update(Settings.DT);
        feederFollower.update(Settings.DT);

        SmartDashboard.putNumber("Feeder/Leader Current", feederLeader.getMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Follower Current", feederFollower.getMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Leader Voltage", feederLeader.getMotor().getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Follower Voltage", feederFollower.getMotor().getMotorVoltage().getValueAsDouble());

        RobotVisualizer.getInstance().updateFeeder(getCurrentRPM());
    }
}