/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

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
import com.stuypulse.robot.util.simulation.RobotVisualizer;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeSim extends Intake {

    // private final SingleJointedArmSim pivotSim;

    // private final TalonFXSimulation pivotMotor;

    // private final PositionTorqueCurrentFOC pivotController;

    // private final TalonFXSimulation rollerMotorLeft;

    // private final TalonFXSimulation rollerMotorRight;

    // private final DCMotorSim rollerSim;

    // private final DutyCycleOut rollerController;

    // private final Follower followerController;

    // private Optional<Voltage> pivotVoltageOverride;

    // private Rotation2d zeroOffset;

    public IntakeSim() {
        // pivotSim = new SingleJointedArmSim(
        //         LinearSystemId.createDCMotorSystem(
        //                 DCMotor.getKrakenX60(1),
        //                 Settings.Intake.Pivot.MOI.in(KilogramSquareMeters),
        //                 Settings.Intake.Pivot.GEAR_RATIO),
        //         DCMotor.getKrakenX60(1),
        //         Settings.Intake.Pivot.GEAR_RATIO,
        //         Settings.Intake.Pivot.PIVOT_ARM_LENGTH.in(Meters),
        //         Settings.Intake.Pivot.MAX_ANGLE.in(Radians),
        //         Settings.Intake.Pivot.MIN_ANGLE.in(Radians), // reversed because negative?
        //         true,
        //         Settings.Intake.Pivot.INITIAL_ANGLE.in(Radians));
        // pivotMotor = new TalonFXSimulation(Ports.Intake.INTAKE_PIVOT_MOTOR, pivotSim);
        // pivotMotor.configure(Motors.Intake.PIVOT_CONFIG);
        // pivotController = new PositionTorqueCurrentFOC(getState().getTargetAngle());
        // rollerSim = new DCMotorSim(
        //         LinearSystemId.createDCMotorSystem(
        //                 DCMotor.getKrakenX60(2),
        //                 Settings.Intake.Roller.J.in(KilogramSquareMeters),
        //                 Settings.Intake.Roller.GEAR_RATIO),
        //         DCMotor.getKrakenX60(2));
        // rollerMotorLeft = new TalonFXSimulation(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT, rollerSim);
        // rollerMotorRight = new TalonFXSimulation(Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT, rollerSim);
        // rollerMotorLeft.configure(Motors.Intake.LEFT_ROLLER_CONFIG);
        // rollerMotorRight.configure(Motors.Intake.RIGHT_ROLLER_CONFIG);
        // rollerController = new DutyCycleOut(0).withEnableFOC(true);
        // followerController = new Follower(rollerMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed);
        // rollerMotorRight.setControl(followerController);
        // pivotVoltageOverride = Optional.empty();
        // zeroOffset = new Rotation2d();
    }

    @Override
    public boolean limitSwitchHit() {
        return true;
    }
    
    @Override
    public Angle getRelativePosition() {
        // return Radians.of(pivotSim.getAngleRads() + zeroOffset.getRadians());
        return Radians.of(0);
    }

    @Override
    public void seedPivotAngle(Angle angle) {
        // zeroOffset = new Rotation2d(-pivotSim.getAngleRads()).plus(new Rotation2d(angle));
        // zeroOffset = new Rotation2d(0, 0);
    }

    @Override
    public AngularVelocity getRollerVelocity() {
        // return rollerMotorRight.getVelocity().getValue();
        return RadiansPerSecond.of(2);
    }

    @Override
    protected void stopRollerMotors() {
        // rollerMotorLeft.stopMotor();
        // rollerMotorRight.stopMotor();
        // re-add the follow control after stopMotor removes it
        // rollerMotorRight.setControl(followerController);
    }

    @Override
    protected void stopPivotMotor() {
        // pivotMotor.stopMotor();
    }

    @Override
    public void setPivotVoltageOverride(Voltage voltage) {
        // this.pivotVoltageOverride = Optional.of(voltage);
    }

    @Override
    public void periodic() {
        // if (!Settings.EnabledSubsystems.INTAKE.get()) {
        //     stopAllMotors();

        //     return;
        // }

        // if (pivotVoltageOverride.isPresent()) {
        //     pivotMotor.setControl(new VoltageOut(pivotVoltageOverride.get()));
        //     return;
        // }
        
        // pivotMotor.setControl(pivotController.withPosition(getState().getTargetAngle().times(-1)).withSlot(0)); // cooked inversion


        // rollerMotorLeft.setControl(rollerController.withOutput(getState().getTargetDutyCycle()));
        
        
        // // all current measured in amps
        // DogLog.log("Intake/Pivot/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        // DogLog.log("Intake/Pivot/Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        // DogLog.log("Intake/Pivot/Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        // DogLog.log("Intake/Rollers/Left Current", rollerMotorLeft.getStatorCurrent().getValueAsDouble());
        // DogLog.log("Intake/Rollers/Right Current", rollerMotorRight.getStatorCurrent().getValueAsDouble());
        // DogLog.log("Intake/Rollers/Left Voltage", rollerMotorLeft.getMotorVoltage().getValueAsDouble());
        // DogLog.log("Intake/Rollers/Right Voltage", rollerMotorRight.getMotorVoltage().getValueAsDouble());
        // DogLog.log("Intake/Rollers/Left Stalling", false);
        // // TODO: implement
        // DogLog.log("Intake/Rollers/Right Stalling", false);
            
        // rollerMotorLeft.update(Settings.DT);
        // rollerMotorRight.update(Settings.DT);
        // // pivotMotor.update(Settings.DT);
        // RobotVisualizer.getInstance().updateIntake(Radians.of(getRelativePosition().in(Radians)), getRollerVelocity());
        // super.periodic();
    }

    @Override
    public SysIdRoutine getIntakeSysIdRoutine() {
    //     return SysId.getRoutine(
    //             Settings.Intake.Pivot.RAMP_RATE,
    //             Settings.Intake.Pivot.STEP_VOLTAGE,
    //             "Intake",
    //             voltage -> setPivotVoltageOverride(voltage),
    //             () -> Radians.of(pivotSim.getAngleRads()),
    //             () -> RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()),
    //             () -> Volts.of(pivotSim.getInput(0)),
    //             getInstance());
        return SysId.getRoutine(null, null, getSubsystem(), null, null, null, null, getInstance());
    }
}
