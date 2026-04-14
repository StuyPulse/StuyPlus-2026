package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSim extends Feeder {
    private final FlywheelSim feeder;
    // private final SimpleMotorFeedforward feedForward;
    // private final PIDController controller;

    public FeederSim() {
        super();

        DCMotor motor = DCMotor.getKrakenX60(2);
        feeder = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                motor,
                0.1,
                3.0
            ),
            motor
        );

        // feedForward = new SimpleMotorFeedforward(Gains.Feeder.kS, Gains.Feeder.kV.getAsDouble(), Gains.Feeder.kA.getAsDouble());
        // controller = new PIDController(Gains.Feeder.kP, Gains.Feeder.kI, Gains.Feeder.kD);
    }

    public double getFeederRPM() {
        return feeder.getAngularVelocityRPM();
    }

    @Override
    public void periodic() {
        super.periodic();

        feeder.setInputVoltage(getState().getTargetDutyCycle() * 12);

        // feedForward.setKv(Gains.Feeder.kV.getAsDouble());
        // feedForward.setKa(Gains.Feeder.kA.getAsDouble());

        // double velocity = getState().getTargetRPM();
        // double voltage = feedForward.calculate(velocity) + controller.calculate(getFeederRPM(), velocity);

        // feeder.setInputVoltage(voltage);
        // feeder.update(Settings.DT);

        // SmartDashboard.putNumber("Feeder/TargetRPM", getState().getTargetRPM());
        // SmartDashboard.putNumber("Feeder/RPM", getFeederRPM());
        // SmartDashboard.putNumber("Feeder/CalculatedRadPerSec", velocity);
        // SmartDashboard.putNumber("Feeder/CalculatedVoltage", voltage);

        RobotVisualizer.getInstance().updateFeeder(feeder.getAngularVelocityRPM());
    }
}