package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Ports.ShooterPorts;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.util.ShooterInterpolation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.util.FerryInterpolation;

public class ShooterImpl extends Shooter {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX shooterMotor3;
    private final TalonFX bottomMotor1;
    private final TalonFX bottomMotor2;
    private final CommandSwerveDrivetrain drivetrain;

    public ShooterImpl() {
        shooterMotor1 = new TalonFX(ShooterPorts.SHOOTER_MOTOR_1);
        shooterMotor2 = new TalonFX(ShooterPorts.SHOOTER_MOTOR_2);
        shooterMotor3 = new TalonFX(ShooterPorts.SHOOTER_MOTOR_3);

        bottomMotor1 = new TalonFX(ShooterPorts.BOTTOM_MOTOR_1);
        bottomMotor2 = new TalonFX(ShooterPorts.BOTTOM_MOTOR_2);

        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor1);
        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor2);
        Motors.Shooter.MOTOR_CONFIG.configure(shooterMotor3);
        Motors.Shooter.MOTOR_CONFIG.configure(bottomMotor1);
        Motors.Shooter.MOTOR_CONFIG.configure(bottomMotor2);

        shooterMotor2.setControl(new Follower(ShooterPorts.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));
        shooterMotor3.setControl(new Follower(ShooterPorts.SHOOTER_MOTOR_1, MotorAlignmentValue.Opposed));

        bottomMotor2.setControl(new Follower(ShooterPorts.BOTTOM_MOTOR_1, MotorAlignmentValue.Opposed));

        drivetrain = CommandSwerveDrivetrain.getInstance();
    }

    public Translation2d getDrivetrainPosition() {
        return drivetrain.getPose().getTranslation();
    }

    public double getShootSpeed(){
        return ShooterInterpolation.getRPM(getDrivetrainPosition().getDistance(Field.getAllianceHubPose().getTranslation()));
    }

    public double getFerrySpeed(){
        return FerryInterpolation.getRPM(getDrivetrainPosition().getDistance(Field.getFerryZonePose(getDrivetrainPosition()).getTranslation())); 
    }

    public void setVoltagesBasedOnState() {
        double targetRPS = getState().getTargetRPM() / 60;

        shooterMotor1.setControl(new VelocityVoltage(targetRPS)); // TODO: periodic stuff

        bottomMotor1.setControl(new VelocityVoltage(targetRPS));
    }

    @Override
    public void periodic() {
        super.periodic();

        setVoltagesBasedOnState();

        SmartDashboard.putNumber("Shooter/Target RPM", getState().getTargetRPM());
        SmartDashboard.putNumber("Shooter/Current RPM", shooterMotor1.getVelocity().getValueAsDouble() * 60);
    }
}
