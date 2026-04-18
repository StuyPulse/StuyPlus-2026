package com.stuypulse.robot.subsystems.shooter;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterSim extends Shooter {
    private final DCMotorSim handoffsim;
    private final DCMotorSim shootersim;
    private final TalonFXSimulation handoffleader;
    private final TalonFXSimulation shooterleader;
    private final TalonFXSimulation shooterfollower;
    private final VelocityVoltage shootercontroller;
    private final DutyCycleOut handoffcontroller;

    public ShooterSim() {
        shootersim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(4),
            Settings.Shooter.J_KG_METERS_SQUARED,
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(4)
        );
        shooterleader = new TalonFXSimulation(shootersim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);
        shooterfollower = new TalonFXSimulation(shootersim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);

        shooterfollower.setControl(new Follower(shooterleader.getMotor().getDeviceID(), MotorAlignmentValue.Aligned));
        shootercontroller = new VelocityVoltage(0);
        shooterleader.setControl(shootercontroller);
        shooterleader.update(Settings.DT);
        shooterfollower.update(Settings.DT);
       
        handoffsim= new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            Settings.Shooter.J_KG_METERS_SQUARED,
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(1)
        );
        handoffleader = new TalonFXSimulation(handoffsim).configure(Motors.Shooter.BOTTOM_MOTOR_CONFIG);
        handoffcontroller = new DutyCycleOut(0);
        handoffleader.setControl(handoffcontroller);
        handoffleader.update(Settings.DT);
    }
}
