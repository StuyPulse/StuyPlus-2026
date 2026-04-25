package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.simulation.SimulationConstants;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeSim extends Intake {
    private final SingleJointedArmSim pivotSim;
    private final TalonFXSimulation pivotMotor;
    private final PositionTorqueCurrentFOC pivotController;

    private final TalonFXSimulation rollerMotor;
    private final TalonFXSimulation rollerFollower;
    private final DCMotorSim rollerSim;
    private final DutyCycleOut rollerController;
    private final Follower rollerFollowerController;
    private Optional<Voltage> pivotVoltageOverride;


    private Rotation2d zeroOffset;

    public IntakeSim() {
        pivotSim = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                Settings.Intake.Pivot.J.in(KilogramSquareMeters),
                Settings.Intake.Pivot.GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            Settings.Intake.Pivot.GEAR_RATIO,
            SimulationConstants.Intake.PIVOT_ARM_LENGTH,
            Settings.Intake.Pivot.MIN_ANGLE.getRadians(),
            Settings.Intake.Pivot.MAX_ANGLE.getRadians(),
            true,
            Settings.Intake.Pivot.INITIAL_ANGLE.getRadians()
        );
        pivotMotor = new TalonFXSimulation(Ports.Intake.MOTOR_INTAKE_PIVOT, pivotSim);
        pivotMotor.configure(Motors.Intake.PIVOT_CONFIG);
        pivotController = new PositionTorqueCurrentFOC(getState().getTargetAngle().getRotations());

        rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2),
                Settings.Intake.Roller.J.in(KilogramSquareMeters),
                Settings.Intake.Roller.GEAR_RATIO
            ),
            DCMotor.getKrakenX60(2)
        );
        rollerMotor = new TalonFXSimulation(Ports.Intake.MOTOR_INTAKE_ROLLER_LEFT, rollerSim);
        rollerFollower = new TalonFXSimulation(Ports.Intake.MOTOR_INTAKE_ROLLER_RIGHT, rollerSim);
        rollerMotor.configure(Motors.Intake.LEFT_ROLLER_CONFIG);
        rollerFollower.configure(Motors.Intake.RIGHT_ROLLER_CONFIG);

        rollerController = new DutyCycleOut(0)
            .withEnableFOC(true);
        rollerFollowerController = new Follower(rollerMotor.getDeviceID(), MotorAlignmentValue.Opposed);
        rollerFollower.setControl(rollerFollowerController);
        pivotVoltageOverride = Optional.empty();
        
        zeroOffset = new Rotation2d();
    }

    @Override
    public Rotation2d getRelativePosition() {
        return new Rotation2d(pivotSim.getAngleRads() + zeroOffset.getRadians());
    }

    private void setZeroOffset(Rotation2d offset) {
        zeroOffset = offset;
    }

    @Override
    public Trigger pivotStalling() {
        return new Trigger(() -> false);
    }

    @Override
    public Command setPivotZero() {
        return Commands.runOnce(() -> setZeroOffset(new Rotation2d(pivotSim.getAngleRads())));
    }
    
    @Override
    public Command setPivotZeroAtBottom() {
        return Commands.runOnce(() -> setZeroOffset(new Rotation2d(-pivotSim.getAngleRads()).plus(Settings.Intake.Pivot.DOWN_ANGLE)));
    }

    @Override
    public double getRollerRPM() {
        return rollerMotor.getVelocity().getValue().in(RPM);
    }

    @Override
    protected void stopMotors() {
        pivotMotor.stopMotor();
        rollerMotor.stopMotor();
        rollerFollower.stopMotor();
        rollerFollower.setControl(rollerFollowerController); // re-add the follow control after stopMotor removes it
    }

    @Override
    public void setPivotVoltageOverride(Voltage voltage) {
        this.pivotVoltageOverride = Optional.of(voltage);
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            stopMotors();
            return;
        }

        if (pivotVoltageOverride.isPresent()) {
            pivotMotor.setControl(new VoltageOut(pivotVoltageOverride.get()));
            pivotMotor.update(Settings.DT);
            return;
        }

        rollerMotor.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));

        rollerMotor.update(Settings.DT);
        rollerFollower.update(Settings.DT);

        pivotMotor.setControl(pivotController.withPosition(getState().getTargetAngle().getRotations()));
        pivotMotor.update(Settings.DT);

        SmartDashboard.putNumber("Intake/Pivot/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble()); // all current measured in amps
        SmartDashboard.putNumber("Intake/Pivot/Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Intake/Rollers/Left Current", rollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Right Current", rollerFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Left Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Right Voltage", rollerFollower.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Rollers/Left Stalling", false);
        SmartDashboard.putBoolean("Intake/Rollers/Right Stalling", false); // TODO: implement

        super.periodic();
    }
    
    @Override
    public SysIdRoutine getIntakeSysIdRoutine() {
        return SysId.getRoutine(Settings.Intake.Pivot.RAMP_RATE,
                Settings.Intake.Pivot.STEP_VOLTAGE,
                "Intake",
                voltage -> setPivotVoltageOverride(voltage),
                () -> Radians.of(pivotSim.getAngleRads()),
                () -> RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()),
                () -> Volts.of(pivotSim.getInput(0)),
                getInstance());
    }
}