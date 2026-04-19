package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederImpl extends Feeder {
    private final TalonFX feederLeader;
    private final TalonFX feederFollower;
    private final DutyCycleOut controller;

    public FeederImpl() {
        feederLeader = new TalonFX(Ports.Feeder.FEEDER_MOTOR_1, Settings.CANIVORE);
        feederFollower = new TalonFX(Ports.Feeder.FEEDER_MOTOR_2, Settings.CANIVORE);

        Motors.Feeder.LEADER_CONFIG.configure(feederLeader);
        Motors.Feeder.FOLLOWER_CONFIG.configure(feederFollower);

        controller = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);

        feederFollower.setControl(new Follower(Ports.Feeder.FEEDER_MOTOR_1, MotorAlignmentValue.Opposed)); // TODO: figure out motor alignment
    }

    @Override
    public double getCurrentRPM() {
         return feederLeader.getVelocity().getValueAsDouble() * 60.0;
    }

    private void stopMotors() {
        feederLeader.stopMotor();
        feederFollower.stopMotor();
        feederFollower.setControl(new Follower(Ports.Feeder.FEEDER_MOTOR_1, MotorAlignmentValue.Opposed)); // make sure this matches follower control in constructor
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!EnabledSubsystems.FEEDER.get()) {
            stopMotors();
            return;
        }
        // Stop shooting if not aligned  
        final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        final Shooter shooter = Shooter.getInstance();

        if (!(swerve.isAlignedToTarget(Field.getHubPose())) && shooter.getState() == ShooterState.SHOOT) {
            setState(FeederState.STOP);
        }

        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation()))) && shooter.getState() == ShooterState.FERRY) {
            setState(FeederState.STOP);
        }

        // Apply
        feederLeader.setControl(controller.withOutput(getState().getTargetDutyCycle()));

        // Logging
        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Feeder/Leader Current", feederLeader.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Feeder/Follower Current", feederFollower.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Feeder/Leader Voltage", feederLeader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Feeder/Follower Voltage", feederFollower.getMotorVoltage().getValueAsDouble());
        }
    }
}