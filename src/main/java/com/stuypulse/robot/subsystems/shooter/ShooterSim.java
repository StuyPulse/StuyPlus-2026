package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim extends Shooter {
    private final FlywheelSim shooterSim;
    private final TalonFXSimulation shooterLeader;
    private final TalonFXSimulation shooterFollower1;
    private final TalonFXSimulation shooterFollower2;
    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower shooterFollowerController;

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

        shooterFollowerController = new Follower(shooterLeader.getMotor().getDeviceID(), MotorAlignmentValue.Opposed);
        shooterFollower1.setControl(shooterFollowerController);
        shooterFollower2.setControl(shooterFollowerController);
        shooterController = new VelocityTorqueCurrentFOC(getState().getTargetAngularVelocity());
    }

    @Override
    public AngularVelocity getCurrentAngularVelocity() {
        return shooterLeader.getMotor().getVelocity().getValue();
    }

    @Override
    protected void stopMotors() {
        shooterLeader.stopMotor();
        shooterFollower1.stopMotor();
        shooterFollower2.stopMotor();

        shooterFollower1.setControl(shooterFollowerController);
        shooterFollower2.setControl(shooterFollowerController);
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.SHOOTER.get()) {
            stopMotors();
            return;
        }

        shooterLeader.setControl(shooterController.withVelocity(getState().getTargetAngularVelocity().in(RotationsPerSecond)));
        shooterLeader.update(Settings.DT);
        shooterFollower1.update(Settings.DT);
        shooterFollower2.update(Settings.DT);

        RobotVisualizer.getInstance().updateShooter(getCurrentAngularVelocity());

        super.periodic();
    }
}
