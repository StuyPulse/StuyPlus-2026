package com.stuypulse.robot.subsystems.handoff;
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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HandoffImpl extends Handoff {
    private final TalonFX handoffMotor;
    private final DutyCycleOut controller;
    
    public HandoffImpl() {
        handoffMotor = new TalonFX(Ports.Handoff.HANDOFF_MOTOR, Settings.CANIVORE);
        controller = new DutyCycleOut(getState().getTargetDutyCycle()).withEnableFOC(true);

        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);
        handoffMotor.setControl(new DutyCycleOut(0));
    }

    @Override
    public AngularVelocity getCurrentAngularVelocity() {
        return handoffMotor.getVelocity().getValue();
    }   

    @Override
    protected void stopMotors() {
        handoffMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (!EnabledSubsystems.HANDOFF.get()) {
            stopMotors();
            return;
        }
    
        // Stop handoffing if not aligned  
        final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        final Shooter shooter = Shooter.getInstance();

        if (!(swerve.isAlignedToTarget(Field.getHubPose())) && shooter.getState() == ShooterState.SHOOT) {
            setState(HandoffState.STOP);
        }

        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation()))) && shooter.getState() == ShooterState.FERRY) {
            setState(HandoffState.STOP);
        }

        // Apply
        handoffMotor.setControl(controller.withOutput(getState().getTargetDutyCycle()));

        // Logging
        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Handoff Current", handoffMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Handoff Voltage", handoffMotor.getMotorVoltage().getValueAsDouble());
        }

        super.periodic();
    }
}

    
