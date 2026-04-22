package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class HandoffImpl extends Handoff {
    private final TalonFX handoffMotor;
    private final DutyCycleOut handoffController;
    public HandoffImpl() {
        handoffMotor = new TalonFX(Ports.ShooterPorts.HANDOFF_MOTOR, Settings.CANIVORE);
        handoffController = new DutyCycleOut(getState().getHandoffDutyCycle()).withEnableFOC(true);

        //configuring
        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);

    }
    @Override
    protected void stopMotors() {
        handoffMotor.stopMotor();
    }
    @Override
    public void periodic() {
        final DutyCycleOut handoffControl = handoffController.withOutput(getState().getHandoffDutyCycle());
        handoffMotor.setControl(handoffControl);
        final Shooter shooter = Shooter.getInstance();
        final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        //turn ts off after aligning to hub!!!! check for ferring, SET TO IDLE
        if (!(swerve.isAlignedToTarget(Field.getHubPose())) && shooter.getState() == ShooterState.SHOOT) {
            setState(HandoffState.IDLE);
        }
        if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation()))) && 
        shooter.getState() == ShooterState.FERRY) {
            setState(HandoffState.IDLE);
        }
        this.logMotor("Handoff", handoffMotor);
        super.periodic();
    }
}