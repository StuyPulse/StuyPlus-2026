package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.RobotVisualizer;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.simulation.SimulationConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeSim extends Intake {
    private final SingleJointedArmSim pivotSim;
    private final TalonFX pivotMotor;
    private final PIDController pivotController;

    private final TalonFX rollerMotor;
    private final TalonFX rollerFollower;
    private final DCMotor rollerGearbox;
    private final DCMotorSim rollerSim;
    private final DutyCycleOut rollerController;

    private Rotation2d zeroOffset;

    public IntakeSim() {
        pivotMotor = new TalonFX(0);
        pivotController = new PIDController(
            Gains.Intake.kP.get(),
            Gains.Intake.kI.get(),
            Gains.Intake.kD.get()
        );

        rollerMotor = new TalonFX(1);
        rollerFollower = new TalonFX(2);
        rollerFollower.setControl(new Follower(rollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        rollerGearbox = DCMotor.getKrakenX60(2);
        rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                rollerGearbox,
                Settings.Intake.ROLLER_J_KG_METERS_SQUARED,
                Settings.Intake.ROLLER_GEAR_RATIO
            ),
            rollerGearbox
        );
        rollerController = new DutyCycleOut(0)
            .withEnableFOC(true);

        DCMotor pivotGearbox = DCMotor.getKrakenX60(1);
        pivotSim = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                pivotGearbox,
                Settings.Intake.J_KG_METERS_SQUARED,
                Settings.Intake.PIVOT_GEAR_RATIO),
            pivotGearbox,
            Settings.Intake.PIVOT_GEAR_RATIO,
            SimulationConstants.Intake.PIVOT_ARM_LENGTH,
            Settings.Intake.PIVOT_MIN_ANGLE,
            Settings.Intake.PIVOT_MAX_ANGLE,
            true,
            Settings.Intake.PIVOT_INITIAL_ANGLE.getRadians()  
        );
        
        zeroOffset = new Rotation2d();
    }

    @Override
    public Rotation2d getRelativePosition() {
        return new Rotation2d(pivotSim.getAngleRads() + zeroOffset.getRadians());
    }

    @Override
    public void setPivotZero() {
        zeroOffset = new Rotation2d(-pivotSim.getAngleRads());
    }
    
    @Override
    public void setPivotZeroAtBottom() {
        zeroOffset = new Rotation2d(-pivotSim.getAngleRads()).plus(new Rotation2d(Math.toRadians(122)));
    }

    @Override
    public double getRollerRPM() {
        return rollerMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            pivotMotor.getSimState().setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()) * Settings.Intake.PIVOT_GEAR_RATIO);
            rollerMotor.setControl(rollerController.withOutput(0));
            return;
        }

        rollerMotor.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));

        final TalonFXSimState rollerState = rollerMotor.getSimState();
        rollerSim.setInputVoltage(rollerState.getMotorVoltage());
        rollerSim.update(Settings.DT);

        TalonFXSimState followerState = rollerFollower.getSimState();
        final double rotorPositionRotations = Units.radiansToRotations(rollerSim.getAngularPositionRad()) * Settings.Intake.ROLLER_GEAR_RATIO;
        final double rollerRotorRPS = rollerSim.getAngularVelocityRPM() / 60.0 * Settings.Intake.ROLLER_GEAR_RATIO;

        followerState.setRawRotorPosition(rotorPositionRotations);
        followerState.setRotorVelocity(rollerRotorRPS);
        rollerState.setRawRotorPosition(rotorPositionRotations);
        rollerState.setRotorVelocity(rollerRotorRPS);

        double voltage = pivotController.calculate(getRelativePosition().getRadians(), getState().getTargetAngle().getRadians());
        pivotSim.setInputVoltage(voltage);
        pivotSim.update(Settings.DT);

        TalonFXSimState pivotState = pivotMotor.getSimState();
        pivotState.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()) * Settings.Intake.PIVOT_GEAR_RATIO);
        pivotState.setRotorVelocity(Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocityRadPerSec()) / 60.0 * Settings.Intake.PIVOT_GEAR_RATIO);
    }
}