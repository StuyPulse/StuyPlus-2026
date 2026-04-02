package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import javax.swing.text.Position;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.EnabledSubsystems;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeImpl extends Intake {

    private final TalonFX intakePivotMotor;
    private final TalonFX intakeRollerMotorLeft;
    private final TalonFX intakeRollerMotorRight;

    private final DutyCycleOut rollerController;
    private final PositionVoltage pivotController;
    private final Follower followerController;

    private final BStream pivotStalling;
    private Optional<Double> pivotVoltageOverride;

    public IntakeImpl() {
        intakePivotMotor = new TalonFX(Ports.Intake.MOTOR_INTAKE_PIVOT, Settings.CANIVORE);
        Motors.Intake.PIVOT_CONFIG.configure(intakePivotMotor);
        intakePivotMotor.setPosition(Settings.Intake.PIVOT_INITIAL_ANGLE.getRotations());

        intakeRollerMotorLeft = new TalonFX(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, Settings.CANIVORE);
        intakeRollerMotorRight = new TalonFX(Ports.Intake.MOTOR_INTAKE_ROLLER_RIGHT, Settings.CANIVORE);

        Motors.Intake.LEFT_ROLLER_CONFIG.configure(intakeRollerMotorLeft);
        Motors.Intake.RIGHT_ROLLER_CONFIG.configure(intakeRollerMotorRight);


        rollerController = new DutyCycleOut(getState().getTargetDutyCycle());
        pivotController = new PositionVoltage(getState().getTargetAngle().getRotations());
        followerController = new Follower(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, MotorAlignmentValue.Opposed);

        intakeRollerMotorRight.setControl(followerController);

        pivotStalling = BStream.create(() -> intakePivotMotor.getSupplyCurrent().getValueAsDouble() > Settings.Intake.PIVOT_STALL_VOLTAGE)
            .filtered(new BDebounce.Both(Settings.Intake.PIVOT_STALL_DEBOUNCE_SEC));
        
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
    public void setPivotZero() {
        intakePivotMotor.setPosition(0);
    }

    @Override
    public void setPivotZeroAtBottom() {
        intakePivotMotor.setPosition(122);
    }

    private boolean pivotIsStalling() {
        return pivotStalling.get();
    }

    @Override
    public void periodic() {
        Boolean pushingDown = false;
        super.periodic();
        if (EnabledSubsystems.INTAKE.get()) {
            if (pivotVoltageOverride.isPresent()) {
                intakePivotMotor.setVoltage(pivotVoltageOverride.get());
            } else {
                Boolean pivotAboveThreshold = intakePivotMotor.getPosition().getValueAsDouble() > Settings.Intake.PUSHDOWN_THRESHOLD.getRotations();

                if((getState() == IntakeState.INTAKE || getState() == IntakeState.OUTTAKE || getState() == IntakeState.DOWN) 
                && pivotAboveThreshold) {
                    intakePivotMotor.setControl(new VoltageOut(0));

                    pushingDown = true;
                } else if (getState() == IntakeState.HOMING) {
                    // intakePivotMotor.setControl(new VoltageOut(Settings.Intake.HOMING_VOLTAGE));
                    intakePivotMotor.setPosition(122);
                } else {
                    intakePivotMotor.setControl(pivotController.withPosition(getState().getTargetAngle().getRotations()));
                    pushingDown = false;
                }

                if (pivotAboveThreshold) {
                    intakeRollerMotorLeft.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));
                } else {
                    intakeRollerMotorLeft.setControl(rollerController.withOutput(0));
                }

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
