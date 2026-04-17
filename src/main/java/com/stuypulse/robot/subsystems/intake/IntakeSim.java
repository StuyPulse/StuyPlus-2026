package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.SimulationConstants;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSim extends Intake {
    private final SingleJointedArmSim pivotSim;
    private final TalonFX pivotMotor;
    private final PIDController pivotController;

    private final TalonFXSimulation rollerMotor;
    private final TalonFXSimulation rollerFollower;
    private final DCMotorSim rollerSim;
    private final DutyCycleOut rollerController;

    private Rotation2d zeroOffset;

    public IntakeSim() {
        pivotMotor = new TalonFX(TalonFXSimulation.getID());
        pivotController = new PIDController(
            Gains.Intake.kP.get(),
            Gains.Intake.kI.get(),
            Gains.Intake.kD.get()
        );

        rollerSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2),
                Settings.Intake.Roller.J_KG_METERS_SQUARED,
                Settings.Intake.Roller.GEAR_RATIO
            ),
            DCMotor.getKrakenX60(2)
        );
        rollerMotor = new TalonFXSimulation(rollerSim).configure(Motors.Intake.LEFT_ROLLER_CONFIG);
        rollerFollower = new TalonFXSimulation(rollerSim).configure(Motors.Intake.RIGHT_ROLLER_CONFIG);
        rollerFollower.setControl(new Follower(rollerMotor.getMotor().getDeviceID(), MotorAlignmentValue.Opposed));
        rollerController = new DutyCycleOut(0)
            .withEnableFOC(true);

        pivotSim = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                Settings.Intake.Pivot.J_KG_METERS_SQUARED,
                Settings.Intake.Pivot.GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            Settings.Intake.Pivot.GEAR_RATIO,
            SimulationConstants.Intake.PIVOT_ARM_LENGTH,
            Math.toRadians(Settings.Intake.Pivot.MIN_ANGLE),
            Math.toRadians(Settings.Intake.Pivot.MAX_ANGLE),
            true,
            Settings.Intake.Pivot.INITIAL_ANGLE.getRadians()  
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
        zeroOffset = new Rotation2d(-pivotSim.getAngleRads()).plus(Settings.Intake.Pivot.DOWN_ANGLE);
    }

    @Override
    public double getRollerRPM() {
        return rollerMotor.getMotor().getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            pivotMotor.getSimState().setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()) * Settings.Intake.Pivot.GEAR_RATIO);
            rollerMotor.setControl(rollerController.withOutput(0));
            return;
        }

        rollerMotor.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));

        rollerMotor.update(Settings.DT);
        rollerFollower.update(Settings.DT);

        double voltage = pivotController.calculate(getRelativePosition().getRadians(), getState().getTargetAngle().getRadians());
        pivotSim.setInputVoltage(voltage);
        pivotSim.update(Settings.DT);

        TalonFXSimState pivotState = pivotMotor.getSimState();
        pivotState.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()));
        pivotState.setRotorVelocity(Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocityRadPerSec()) / 60);

        SmartDashboard.putNumber("Intake/Pivot/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble()); // all current measured in amps
        SmartDashboard.putNumber("Intake/Pivot/Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());

        final TalonFX rollerRealLeader = rollerMotor.getMotor();
        final TalonFX rollerRealFollower = rollerFollower.getMotor();
        SmartDashboard.putNumber("Intake/Rollers/Left Current", rollerRealLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Right Current", rollerRealFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Left Voltage", rollerRealLeader.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Rollers/Right Voltage", rollerRealFollower.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Rollers/Left Stalling", false);
        SmartDashboard.putBoolean("Intake/Rollers/Right Stalling", false); // TODO: implement
    }
}