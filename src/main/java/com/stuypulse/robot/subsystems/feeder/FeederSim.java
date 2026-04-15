package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.constants.Motors;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSim extends Feeder {
    private final TalonFX feederLeader;
    private final TalonFX feederFollower;
    private final DCMotor feederGearbox;
    private final DCMotorSim feederSim;
    private final DutyCycleOut feederController;

    public FeederSim() {
        feederLeader = new TalonFX(1);
        feederFollower = new TalonFX(2);

        Motors.Feeder.LEADER_CONFIG.configure(feederLeader);
        Motors.Feeder.FOLLOWER_CONFIG.configure(feederFollower);

        feederFollower.setControl(new Follower(feederLeader.getDeviceID(), MotorAlignmentValue.Opposed));
        feederGearbox = DCMotor.getKrakenX60(2);
        feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                feederGearbox,
                Settings.Feeder.J_KG_METERS_SQUARED,
                Settings.Feeder.GEAR_RATIO
            ),
            feederGearbox
        );

        feederController = new DutyCycleOut(0)
            .withEnableFOC(true);
    }

    @Override
    public double getCurrentRPM() {
         return feederLeader.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            feederLeader.setControl(feederController.withOutput(0));
            return;
        }

        feederLeader.setControl(feederController.withOutput(1)); // apply control to leader before grabbing state to update other motors

        // State
        final TalonFXSimState feederState = feederLeader.getSimState();
        final TalonFXSimState followerState = feederFollower.getSimState();

        // Apply
        feederSim.setInputVoltage(feederState.getMotorVoltage());
        feederSim.update(Settings.DT);

        final double rotorPositionRotations = Units.radiansToRotations(feederSim.getAngularPositionRad()) * Settings.Feeder.GEAR_RATIO;
        final double feederRotorRPS = feederSim.getAngularVelocityRPM() / 60.0 * Settings.Feeder.GEAR_RATIO;

        followerState.setRawRotorPosition(rotorPositionRotations);
        followerState.setRotorVelocity(feederRotorRPS);
        feederState.setRawRotorPosition(rotorPositionRotations);
        feederState.setRotorVelocity(feederRotorRPS);

        SmartDashboard.putNumber("Feeder/Leader Current", feederLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Follower Current", feederFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Leader Voltage", feederLeader.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Follower Voltage", feederFollower.getMotorVoltage().getValueAsDouble());

        RobotVisualizer.getInstance().updateFeeder(getCurrentRPM());
    }
}