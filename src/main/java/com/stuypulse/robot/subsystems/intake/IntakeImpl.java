package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    private final TalonFX intakeRollerMotorLeft;
    private final TalonFX intakeRollerMotorRight;

    private final DutyCycleOut rollerController;
    private final MotionMagicVoltage pivotController;

    private Optional<Double> pivotVoltageOverride;

    public IntakeImpl() {
        intakePivotMotor = new TalonFX(Ports.Intake.MOTOR_INTAKE_PIVOT, Settings.CANIVORE);
        Motors.Intake.PIVOT_CONFIG.configure(intakePivotMotor);
        intakePivotMotor.setPosition(Settings.Intake.PIVOT_INITIAL_ANGLE.getRotations());

        intakeRollerMotorLeft = new TalonFX(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, Settings.CANIVORE);
        intakeRollerMotorRight = new TalonFX(Ports.Intake.MOTOR_INTAKE_ROLLER_RIGHT, Settings.CANIVORE);

        Motors.Intake.LEFT_ROLLER_CONFIG.configure(intakeRollerMotorLeft);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(intakeRollerMotorRight);

        intakeRollerMotorRight.setControl(new Follower(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, MotorAlignmentValue.Aligned));
        
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
    public double getRollerRPM() {
        return intakeRollerMotorLeft.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    public void periodic() {
        Boolean pushingDown = false;
        super.periodic();
        if (EnabledSubsystems.INTAKE.get()) {
            if (pivotVoltageOverride.isPresent()) {
                intakePivotMotor.setVoltage(pivotVoltageOverride.get());
            } else {
                if((getState() == IntakeState.INTAKE || getState() == IntakeState.OUTTAKE || getState() == IntakeState.DOWN) 
                && intakePivotMotor.getPosition().getValueAsDouble() > Settings.Intake.PUSHDOWN_THRESHOLD.getRotations()) {
                    intakePivotMotor.setControl(new VoltageOut(Settings.Intake.PUSHDOWN_VOLTAGE));
                    pushingDown = true;
                } else {
                    intakePivotMotor.setControl(pivotController.withPosition(getState().getTargetAngle().getRotations()));
                    pushingDown = false;
                }

                intakeRollerMotorLeft.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));
            }
        } else {
            intakePivotMotor.stopMotor();
            intakeRollerMotorLeft.stopMotor();
        }

        SmartDashboard.putNumber("Intake/Roller Current (amps)", intakeRollerMotorLeft.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Voltage", intakeRollerMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Pushing Down", pushingDown);

        SmartDashboard.putNumber("Intake/Roller Duty Cycle", intakeRollerMotorLeft.getDutyCycle().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Pivot Current (amps)", intakePivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Voltage", intakePivotMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Pivot Angle (deg)", getRelativePosition().getDegrees());

        SmartDashboard.putBoolean("Intake/Pivot At Target Angle", atAngle());
    }

    public SysIdRoutine getIntakeSysIdRoutine() {
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
