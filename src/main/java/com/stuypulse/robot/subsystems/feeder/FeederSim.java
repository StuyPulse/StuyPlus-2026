package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSim extends Feeder {
    private final FlywheelSim feeder;
    private final FeederVisualizer visualizer;
    private double velocity;

    public FeederSim() {
        super();

        feeder = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(5),
                1.0,
                3.0
            ),
            DCMotor.getKrakenX44(2)
        );

        visualizer = FeederVisualizer.getInstance();
    }

    public double getFeederRPM() {
        return feeder.getAngularVelocityRPM();
    }

    @Override
    public void periodic() {
        super.periodic();
        velocity = getState().getTargetRPM() * 2 * Math.PI / 60;

        feeder.setAngularVelocity(velocity);
        feeder.update(Settings.DT);
        visualizer.update(feeder.getAngularVelocityRPM());
        SmartDashboard.putNumber("Feeder/TargetRPM", getState().getTargetRPM());
    }
}