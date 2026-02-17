package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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
            Gains.Shooter.kP,
            Gains.Shooter.kI,
            Gains.Shooter.kD
        );
    }

    public double getShooterRPM() {
        return shooter.getAngularVelocityRPM();
    }

    @Override
    public double getShootSpeed() {
        return 25;
    };

    @Override
    public double getFerrySpeed() {
        return 67; // arbitrary speeds because we don't have a drivetrain sim
    };

    @Override
    public void periodic() {
        super.periodic();

        shooter.setInputVoltage(shooterController.calculate(getShooterRPM(), getState().getTargetRPM()));
        shooter.update(Settings.DT); // ooooooohhhhhhhhhhh
        visualizer.update(getShooterRPM());
        SmartDashboard.putNumber("Shooter/TargetRPM", getState().getTargetRPM());
        SmartDashboard.putNumber("Shooter/SimRPM", getShooterRPM());
    }
}