package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;

import edu.wpi.first.units.measure.*;

public abstract class HandoffIOTalonFXBase implements HandoffIO {
    private final TalonFX handoffMotor;

    private final VoltageOut handoffController;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;

    public HandoffIOTalonFXBase(TalonFX motor) {
        handoffMotor = motor;
        handoffController = new VoltageOut(0).withEnableFOC(true);

        Motors.Handoff.HANDOFF_MOTOR_CONFIG.configure(handoffMotor);

        position = handoffMotor.getPosition();
        velocity = handoffMotor.getVelocity();
        voltage = handoffMotor.getMotorVoltage();
        supplyCurrent = handoffMotor.getSupplyCurrent();
        statorCurrent = handoffMotor.getStatorCurrent();
    }

    @Override
    public void stopMotors() {
        handoffMotor.stopMotor();
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        inputs.position = position.refresh().getValue();
        inputs.velocity = velocity.refresh().getValue();
        inputs.voltage = voltage.refresh().getValue();
        inputs.supplyCurrent = supplyCurrent.refresh().getValue();
        inputs.statorCurrent = statorCurrent.refresh().getValue();
        // final Shooter shooter = Shooter.getInstance();
        // final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        // if (!(swerve.isAlignedToTarget(Field.getHubPose()))
        //         && shooter.getState() == ShooterState.SHOOT) {
        //     setState(HandoffState.IDLE);
        // }
        // // TODO: consider relaxing tolerances for ferrying
        // if (!(swerve.isAlignedToTarget(Field.getFerryZonePose(swerve.getPose().getTranslation())))
        //         && shooter.getState() == ShooterState.FERRY) {
        //     setState(HandoffState.IDLE);
        // }

        // Control
        // final Voltage voltage = handoffStalling()
        //         ? Handoff.HandoffState.REVERSE.getTargetVoltage()
        //         : getState().getTargetVoltage();
        // final VoltageOut handoffControl = handoffController.withOutput(voltage);
    }

    public void setTargetVoltage(Voltage voltage) {
        handoffMotor.setControl(handoffController.withOutput(voltage));
    }
}
