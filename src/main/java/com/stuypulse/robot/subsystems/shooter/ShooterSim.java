package com.stuypulse.robot.subsystems.shooter;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
    private final TalonFXSimulation handoffmotor;
    private final TalonFXSimulation shooterleader;
    private final TalonFXSimulation shooterfollower1;
    private final TalonFXSimulation shooterfollower2;
    private final VelocityTorqueCurrentFOC shootercontroller;
    private final DutyCycleOut handoffcontroller;

    public ShooterSim() {
        
        shootersim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(3),
            Settings.Shooter.J_KG_METERS_SQUARED,
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(3)
        );
        shooterleader = new TalonFXSimulation(shootersim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);
        shooterfollower1 = new TalonFXSimulation(shootersim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);
        shooterfollower2 = new TalonFXSimulation(shootersim).configure(Motors.Shooter.SHOOTER_MOTOR_CONFIG);

        shooterfollower1.setControl(new Follower(shooterleader.getMotor().getDeviceID(), MotorAlignmentValue.Opposed));
        shooterfollower2.setControl(new Follower(shooterleader.getMotor().getDeviceID(), MotorAlignmentValue.Opposed));
        shootercontroller = new VelocityTorqueCurrentFOC(getState().getRPM());

       
        handoffsim = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            Settings.Shooter.J_KG_METERS_SQUARED,
            Settings.Shooter.GEAR_RATIO),
            DCMotor.getKrakenX60(1)
        );
        handoffmotor = new TalonFXSimulation(handoffsim).configure(Motors.Shooter.HANDOFF_MOTOR_CONFIG);
        handoffcontroller = new DutyCycleOut(getState().getHandoffMotorDutyCycle());
    }
    @Override
    public void periodic() {
        shooterleader.setControl(shootercontroller.withVelocity(getState().getRPM() / 60));
        shooterleader.update(Settings.DT);
        shooterfollower1.update(Settings.DT);
        shooterfollower2.update(Settings.DT);
        handoffmotor.setControl(handoffcontroller.withOutput(getState().getHandoffMotorDutyCycle()));
        handoffmotor.update(Settings.DT);
        super.periodic();
    }
}
