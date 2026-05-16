/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeSim extends Intake {

    private final SingleJointedArmSim pivotSim;

    private final TalonFXSimulation pivotMotor;

    private final PositionTorqueCurrentFOC pivotController;

    private final TalonFXSimulation rollerMotor;

    private final TalonFXSimulation rollerFollower;

    private final DCMotorSim rollerSim;

    private final DutyCycleOut rollerController;

    private final Follower rollerFollowerController;

    private Rotation2d zeroOffset;

    public IntakeSim() {
        pivotSim = new SingleJointedArmSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        Settings.Intake.Pivot.J.in(KilogramSquareMeters),
                        Settings.Intake.Pivot.GEAR_RATIO),
                DCMotor.getKrakenX60(1),
                Settings.Intake.Pivot.GEAR_RATIO,
                Settings.Intake.Pivot.PIVOT_ARM_LENGTH.in(Meters),
                Settings.Intake.Pivot.MIN_ANGLE.in(Radians),
                Settings.Intake.Pivot.MAX_ANGLE.in(Radians),
                true,
                Settings.Intake.Pivot.INITIAL_ANGLE.in(Radians));
        pivotMotor = new TalonFXSimulation(Ports.Intake.INTAKE_PIVOT_MOTOR, pivotSim);
        pivotMotor.configure(Motors.Intake.PIVOT_CONFIG);
        pivotController = new PositionTorqueCurrentFOC(getState().getTargetAngle().in(Rotations));
        rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(2),
                        Settings.Intake.Roller.J.in(KilogramSquareMeters),
                        Settings.Intake.Roller.GEAR_RATIO),
                DCMotor.getKrakenX60(2));
        rollerMotor = new TalonFXSimulation(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, rollerSim);
        rollerFollower = new TalonFXSimulation(Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT, rollerSim);
        rollerMotor.configure(Motors.Intake.LEFT_ROLLER_CONFIG);
        rollerFollower.configure(Motors.Intake.RIGHT_ROLLER_CONFIG);
        rollerController = new DutyCycleOut(0).withEnableFOC(true);
        rollerFollowerController = new Follower(rollerMotor.getDeviceID(), MotorAlignmentValue.Opposed);
        rollerFollower.setControl(rollerFollowerController);
        zeroOffset = new Rotation2d();
    }

    @Override
    public Angle getRelativePosition() {
        return Radians.of(pivotSim.getAngleRads() + zeroOffset.getRadians());
    }

    @Override
    public void seedPivotAngle(Angle angle) {
        zeroOffset = new Rotation2d(-pivotSim.getAngleRads()).plus(new Rotation2d(angle));
    }

    @Override
    public AngularVelocity getRollerVelocity() {
        return rollerFollower.getVelocity().getValue();
    }

    @Override
    protected void stopMotors() {
        pivotMotor.stopMotor();
        rollerMotor.stopMotor();
        rollerFollower.stopMotor();
        // re-add the follow control after stopMotor removes it
        rollerFollower.setControl(rollerFollowerController);
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.INTAKE.get()) {
            stopMotors();
            return;
        }

        rollerMotor.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));
        rollerMotor.update(Settings.DT);
        rollerFollower.update(Settings.DT);
        pivotMotor.setControl(pivotController.withPosition(getState().getTargetAngle().in(Rotations)));
        pivotMotor.update(Settings.DT);
        // all current measured in amps
        DogLog.log("Intake/Pivot/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("Intake/Pivot/Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        DogLog.log("Intake/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Intake/Rollers/Left Current", rollerMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log(
                "Intake/Rollers/Right Current", rollerFollower.getStatorCurrent().getValueAsDouble());
        DogLog.log("Intake/Rollers/Left Voltage", rollerMotor.getMotorVoltage().getValueAsDouble());
        DogLog.log("Intake/Rollers/Right Voltage", rollerFollower.getMotorVoltage().getValueAsDouble());
        DogLog.log("Intake/Rollers/Left Stalling", false);
        // TODO: implement
        DogLog.log("Intake/Rollers/Right Stalling", false);
        super.periodic();
    }
}
