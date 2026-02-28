package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.FerryInterpolation;
import com.stuypulse.robot.util.ShooterInterpolation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSim extends Shooter {
    private final FlywheelSim shooter;
    private final CommandSwerveDrivetrain drivetrain;

    private final PIDController shooterController;

    public ShooterSim() {
        super();

        drivetrain = CommandSwerveDrivetrain.getInstance();

        shooter = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(5),
                1.0,
                3.0
            ),
            DCMotor.getKrakenX44(2)
        );

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
        return ShooterInterpolation.getRPM(drivetrain.getPose().getTranslation().getDistance(Field.getAllianceHubPose().getTranslation()));
    };

    @Override
    public double getFerrySpeed() {
        return FerryInterpolation.getRPM(drivetrain.getPose().getTranslation().getDistance(Field.getFerryZonePose(drivetrain.getPose().getTranslation()).getTranslation()));
    };

    @Override
    public void periodic() {
        super.periodic();

        shooter.setInputVoltage(shooterController.calculate(getShooterRPM(), getState().getTargetRPM()));
        shooter.update(Settings.DT);
        SmartDashboard.putNumber("Shooter/TargetRPM", getState().getTargetRPM());
        SmartDashboard.putNumber("Shooter/SimRPM", getShooterRPM());
        SmartDashboard.putNumber("Drivetrain/Pose X", drivetrain.getPose().getX());
        SmartDashboard.putNumber("Drivetrain/Pose Y", drivetrain.getPose().getY());
        SmartDashboard.putNumber("Shooter/Hub Distance", drivetrain.getPose().getTranslation().getDistance(Field.getAllianceHubPose().getTranslation()));
        SmartDashboard.putNumber("Shooter/Transformed Distance", drivetrain.getPose().getTranslation().getDistance(Field.blueHubCenter.getTranslation()));
        SmartDashboard.putNumber("Shooter/Ferry Distance", drivetrain.getPose().getTranslation().getDistance(Field.getFerryZonePose(drivetrain.getPose().getTranslation()).getTranslation()));
    }
}