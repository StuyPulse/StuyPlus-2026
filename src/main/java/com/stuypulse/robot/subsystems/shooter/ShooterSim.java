package com.stuypulse.robot.subsystems.shooter;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterSim extends Shooter {
    private final DCMotorSim handoffSim;
    private final DCMotorSim shooterSim;
    private final TalonFXSimulation handoffMotor;
    private final TalonFXSimulation shooterLeader;
    private final TalonFXSimulation shooterFollower1;
    private final TalonFXSimulation shooterFollower2;
    private final VelocityTorqueCurrentFOC shooterController;
    private final DutyCycleOut handoffController;

    public ShooterSim() {
        shooterSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(3),
            Settings.Shooter.J_KG_METERS_SQUARED,
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(3)
        );
        shooterLeader = new TalonFXSimulation(shooterSim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);
        shooterFollower1 = new TalonFXSimulation(shooterSim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);
        shooterFollower2 = new TalonFXSimulation(shooterSim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);

        shooterFollower1.setControl(new Follower(shooterLeader.getMotor().getDeviceID(), MotorAlignmentValue.Opposed));
        shooterFollower2.setControl(new Follower(shooterLeader.getMotor().getDeviceID(), MotorAlignmentValue.Opposed));
        shooterController = new VelocityTorqueCurrentFOC(getState().getRPM());

       
        handoffSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            Settings.Shooter.J_KG_METERS_SQUARED,
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(1)
        );
        handoffMotor = new TalonFXSimulation(handoffSim).configure(Motors.Shooter.HANDOFF_MOTOR_CONFIG);
        handoffController = new DutyCycleOut(getState().getHandoffMotorDutyCycle()).withEnableFOC(true);
    }

    @Override
    public double getCurrentRPM() {
        return shooterLeader.getMotor().getVelocity().getValue().in(RPM);
    }

    @Override
    public void periodic() {
        shooterLeader.setControl(shooterController.withVelocity(getState().getRPM() / 60));
        shooterLeader.update(Settings.DT);
        shooterFollower1.update(Settings.DT);
        shooterFollower2.update(Settings.DT);
        handoffMotor.setControl(handoffController.withOutput(getState().getHandoffMotorDutyCycle()));
        handoffMotor.update(Settings.DT);

        super.periodic();
    }
}
