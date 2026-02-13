package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSim extends Shooter {
    private final FlywheelSim shooter;
    private final ShooterVisualizer visualizer;

    private final PIDController shooterController;

    public ShooterSim() {
        super();

        shooter = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(5),
                1.0,
                3.0
            ),
            DCMotor.getKrakenX44(2)
        );

        visualizer = ShooterVisualizer.getInstance();

        shooterController = new PIDController(
            Settings.Shooter.kP,
            Settings.Shooter.kI,
            Settings.Shooter.kD
        );
    }

    public double getShooterRPM() {
        return shooter.getAngularVelocityRPM();
    }

    @Override
    public double getShootSpeed() {
        return 41;
    };

    @Override
    public double getFerrySpeed() {
        return 41; // arbitrary speeds because we don't have a drivetrain sim
    };

    @Override
    public void periodic() {
        super.periodic();

        shooterController.update(getState().getTargetRPM(), getShooterRPM());
        // SmartDashboard.putNumber("hdsr/Output Voltage", controller);

        shooter.setInputVoltage(shooterController.getOutput());
    }

    @Override
    public void simulationPeriodic(){
        super.simulationPeriodic();

        shooter.update(Settings.DT);
        visualizer.update(getState().getTargetRPM());
        SmartDashboard.putNumber("Shooter/TargetRPM", getState().getTargetRPM());
    }
}