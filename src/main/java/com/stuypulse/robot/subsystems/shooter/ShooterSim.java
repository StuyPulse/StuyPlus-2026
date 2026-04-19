package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim extends Shooter {
    private final DCMotorSim handoffSim;
    private final FlywheelSim shooterSim;
    private final TalonFXSimulation handoffMotor;
    private final TalonFXSimulation shooterLeader;
    private final TalonFXSimulation shooterFollower1;
    private final TalonFXSimulation shooterFollower2;
    private final VelocityTorqueCurrentFOC shooterController;
    private final DutyCycleOut handoffController;

    public ShooterSim() {
        shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(3),
            Settings.Shooter.J.in(KilogramSquareMeters),
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(3)
        );
        shooterLeader = new TalonFXSimulation(shooterSim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);
        shooterFollower1 = new TalonFXSimulation(shooterSim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);
        shooterFollower2 = new TalonFXSimulation(shooterSim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);

        shooterFollower1.setControl(new Follower(shooterLeader.getMotor().getDeviceID(), MotorAlignmentValue.Opposed));
        shooterFollower2.setControl(new Follower(shooterLeader.getMotor().getDeviceID(), MotorAlignmentValue.Opposed));
        shooterController = new VelocityTorqueCurrentFOC(getState().getTargetAngularVelocity());

       
        handoffSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            Settings.Shooter.J.in(KilogramSquareMeters),
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(1)
        );
        handoffMotor = new TalonFXSimulation(handoffSim).configure(Motors.Shooter.HANDOFF_MOTOR_CONFIG);
        handoffController = new DutyCycleOut(getState().getHandoffMotorDutyCycle()).withEnableFOC(true);
    }

    @Override
    public AngularVelocity getCurrentAngularVelocity() {
        return shooterLeader.getMotor().getVelocity().getValue();
    }

    @Override
    public void periodic() {
        shooterLeader.setControl(shooterController.withVelocity(getState().getTargetAngularVelocity().in(RotationsPerSecond)));
        shooterLeader.update(Settings.DT.in(Seconds));
        shooterFollower1.update(Settings.DT.in(Seconds));
        shooterFollower2.update(Settings.DT.in(Seconds));
        handoffMotor.setControl(handoffController.withOutput(getState().getHandoffMotorDutyCycle()));
        handoffMotor.update(Settings.DT.in(Seconds));

        RobotVisualizer.getInstance().updateShooter(getCurrentAngularVelocity());

        super.periodic();
    }
}
