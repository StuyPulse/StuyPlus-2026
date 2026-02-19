        package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.constants.Motors;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    @Override
    public void periodic() {
        
        if (EnabledSubsystems.Intake.get()) {
            if (pivotVoltageOverride.isPresent()) {
                intakePivotMotor.setVoltage(pivotVoltageOverride.get());
            } else {
                intakePivotMotor.setControl(pivotController.withPosition(getState().getTargetAngle().getRotations()));
                intakeRollerMotor.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));
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
}
