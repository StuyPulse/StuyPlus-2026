package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut; 
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.units.measure.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederImpl extends Feeder {
    private final TalonFX feederLeader;
    private final DutyCycleOut controller;

    public FeederImpl() {
        feederLeader = new TalonFX(Ports.Feeder.FEEDER_MOTOR_1, Settings.CANIVORE);

        Motors.Feeder.LEADER_CONFIG.configure(feederLeader);

        controller = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);

    }

    @Override
    public AngularVelocity getCurrentAngularVelocity() {
        return feederLeader.getVelocity().getValue();
    }

    @Override
    protected void stopMotors() {
        feederLeader.stopMotor();
    }

    @Override
    public void periodic() {
        if (!EnabledSubsystems.FEEDER.get()) {
            stopMotors();
            return;
        }
        // Stop shooting if not aligned  
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        Shooter shooter = Shooter.getInstance();

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
            SmartDashboard.putNumber("Feeder/Leader Voltage", feederLeader.getMotorVoltage().getValueAsDouble());
        }

        super.periodic();
    }
}