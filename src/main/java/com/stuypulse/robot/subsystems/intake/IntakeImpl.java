package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeImpl extends Intake {

    private final TalonFX intakePivotMotor;
    private final TalonFX intakeRollerMotor;

    private final DutyCycleOut rollerController;
    private final MotionMagicVoltage pivotController;

    private Optional<Double> pivotVoltageOverride;

    public IntakeImpl() {
        intakePivotMotor = new TalonFX(Ports.Intake.MOTOR_INTAKEPIVOT);
        Motors.Intake.PIVOT_CONFIG.configure(intakePivotMotor);

        intakeRollerMotor = new TalonFX(Ports.Intake.MOTOR_INTAKEROLLER);
        Motors.Intake.ROLLER_CONFIG.configure(intakeRollerMotor);
        intakePivotMotor.setPosition(Settings.Intake.INITIAL_POSITION);

        rollerController = new DutyCycleOut(getState().getTargetDutyCycle());
        pivotController = new MotionMagicVoltage(getState().getTargetAngle().getRotations());

        pivotVoltageOverride = Optional.empty();
    }

    @Override
    public Rotation2d getRelativePosition() {
        return Rotation2d.fromRotations(intakePivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public boolean atAngle() {
        return Math.abs(
                (getRelativePosition().getRotations()) - getState().getTargetAngle().getRotations())
                < Settings.Intake.ANGLE_TOLERANCE.getRotations();
    }

    public void setPivotVoltageOverride(Optional<Double> pivotVoltageOverride) {
        this.pivotVoltageOverride = pivotVoltageOverride;
    }

    @Override
    public void periodic() {

        if (EnabledSubsystems.Intake.get()) {
            if (!pivotVoltageOverride.isPresent()) {
                intakePivotMotor.setControl(pivotController.withPosition(getState().getTargetAngle().getRotations()));
                intakeRollerMotor.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));
            } else {
                intakePivotMotor.setVoltage(pivotVoltageOverride.get());
            }
        } else {
            intakePivotMotor.stopMotor();
            intakeRollerMotor.stopMotor();
        }

        SmartDashboard.putNumber("Intake/Roller Current (amps)", intakeRollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Voltage", intakeRollerMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Pivot Current (amps)", intakePivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Voltage", intakePivotMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putBoolean("Intake/Pivot At Target Angle", atAngle());
    }

    public SysIdRoutine getShooterSysIdRoutine() {
        return SysId.getRoutine(Settings.Intake.RAMP_RATE,
                Settings.Intake.STEP_VOLTAGE,
                "Intake",
                voltage -> setPivotVoltageOverride(Optional.of(voltage)),
                () -> intakePivotMotor.getPosition().getValueAsDouble(),
                () -> intakePivotMotor.getVelocity().getValueAsDouble(),
                () -> intakePivotMotor.getMotorVoltage().getValueAsDouble(),
                getInstance()
        );
    }
}
