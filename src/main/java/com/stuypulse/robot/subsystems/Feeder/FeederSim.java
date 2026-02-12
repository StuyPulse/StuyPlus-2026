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

    private final PIDController feederController;

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

        feederController = new PIDController(
            Settings.Feeder.kP,
            Settings.Feeder.kI,
            Settings.Feeder.kD
        );
    }

    public double getFeederRPM() {
        return feeder.getAngularVelocityRPM();
    }

    @Override
    public void periodic() {
        super.periodic();

        feederController.update(getState().getTargetRPM(), getFeederRPM());
        // SmartDashboard.putNumber("hdsr/Output Voltage", controller);

        feeder.setInputVoltage(feederController.getOutput());
    }

    @Override
    public void simulationPeriodic(){
        super.simulationPeriodic();

        feeder.update(Settings.DT);
        visualizer.update(getState().getTargetRPM());
        SmartDashboard.putNumber("Feeder/TargetRPM", getState().getTargetRPM());
    }
}