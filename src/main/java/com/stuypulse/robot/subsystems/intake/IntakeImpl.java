        package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends Intake {
    private final TalonFX intakePivotMotor;
    private final TalonFX intakeRollerMotor;
    private final PositionVoltage pivotPositionVoltageController;

    public IntakeImpl() {
        intakePivotMotor = new TalonFX(Ports.Intake.MOTOR_INTAKEPIVOT);
        Motors.Intake.PIVOTConfig.configure(intakePivotMotor);
        
        intakeRollerMotor = new TalonFX(Ports.Intake.MOTOR_INTAKEROLLER);   
        Motors.Intake.ROLLERConfig.configure(intakeRollerMotor);  
        intakePivotMotor.setPosition(Settings.Intake.INITIAL_POSITION);

        pivotPositionVoltageController = new PositionVoltage(0.0);
    }  

    @Override
    public Rotation2d getRelativePosition() {
        return Rotation2d.fromRotations(intakePivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {

        currentPivotState.position = getRelativePosition().getRotations();
        currentPivotState.velocity = 0.0;

        targetPivotState.position = getState().getAngle() / 360;
        targetPivotState.velocity = 0.0;
        
        intakePivotMotor.setPosition(Ports.Intake.MOTOR_INTAKEPIVOT);
        intakeRollerMotor.setPosition(Ports.Intake.MOTOR_INTAKEROLLER);

        TrapezoidProfile profile = new TrapezoidProfile(
            new Constraints(Settings.Intake.ROLLER_MAX_VEL, Settings.Intake.ROLLER_MAX_ACCEL)
        );

        TrapezoidProfile.State nextPivot = profile.calculate(
            Settings.DT,
            currentPivotState,
            targetPivotState
        );
        
        intakePivotMotor.setControl(pivotPositionVoltageController.withPosition(nextPivot.position));
        intakeRollerMotor.setVoltage(getState().getVoltage());
      
        SmartDashboard.putNumber("Intake/Roller Current (amps)", intakeRollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Voltage", intakeRollerMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Pivot Current (amps)", intakePivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Voltage", intakePivotMotor.getMotorVoltage().getValueAsDouble());
    }
}
