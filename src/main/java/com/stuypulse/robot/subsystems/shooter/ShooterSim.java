package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSim extends Shooter {
    private final FlywheelSim shooterSim;
    private final TalonFXSimulation shooterLeader;
    private final TalonFXSimulation shooterFollower1;
    private final TalonFXSimulation shooterFollower2;
    private final VelocityTorqueCurrentFOC shooterController;
    private final Follower shooterFollowerController;

    private Optional<Voltage> voltageOverride;

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

       
        handoffSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            Settings.Shooter.J.in(KilogramSquareMeters),
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(1)
        );

        voltageOverride = Optional.empty();
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
    public void setVoltageOverride(Voltage voltage) {
        this.voltageOverride = Optional.of(voltage);
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

    
    public SysIdRoutine getShooterSysIdRoutine() {
        return SysId.getRoutine(Settings.Shooter.RAMP_RATE,
                Settings.Shooter.STEP_VOLTAGE,
                "Shooter",
                voltage -> setVoltageOverride(voltage),
                () -> shooterLeader.getMotor().getPosition().getValue(),
                () -> shooterLeader.getMotor().getVelocity().getValue(),
                () -> shooterLeader.getMotor().getMotorVoltage().getValue(),
                getInstance());
    }
}
