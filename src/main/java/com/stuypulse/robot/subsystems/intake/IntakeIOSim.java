/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeSimulationSettings;

public class IntakeIOSim extends IntakeIOBase {
    private static final SystemSim<SingleJointedArmSim> pivotSim = SystemSim.of(new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1),
                    IntakeConstants.IntakeSettings.Pivot.MOI.in(KilogramSquareMeters),
                    IntakeConstants.IntakeSettings.Pivot.GEAR_RATIO),
            DCMotor.getKrakenX60(1),
            IntakeConstants.IntakeSettings.Pivot.GEAR_RATIO,
            IntakeConstants.IntakeSettings.Pivot.PIVOT_ARM_LENGTH.in(Meters),
            IntakeConstants.IntakeSettings.Pivot.MAX_ANGLE.in(Radians),
            IntakeConstants.IntakeSettings.Pivot.MIN_ANGLE.in(Radians), // reversed because negative?
            true,
            IntakeConstants.IntakeSettings.Pivot.INITIAL_ANGLE.in(Radians)));
    private static TalonFXSimulation getPivotMotor(int id) {
        final TalonFXSimulation pivotMotor = new TalonFXSimulation(id, IntakeConstants.IntakeSettings.Pivot.GEAR_RATIO, pivotSim);
        return pivotMotor;
    }
    
    private static final SystemSim<DCMotorSim> rollerSim = SystemSim.of(new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2),
            IntakeConstants.IntakeSettings.Roller.J.in(KilogramSquareMeters),
            IntakeConstants.IntakeSettings.Roller.GEAR_RATIO),
        DCMotor.getKrakenX60(2)));
    private static TalonFXSimulation getRollerMotor(int id) {
        final TalonFXSimulation rollerMotor = new TalonFXSimulation(id, IntakeConstants.IntakeSettings.Roller.GEAR_RATIO, rollerSim);
        return rollerMotor;
    }

    private final TalonFXSimulation pivotMotor;

    private final TalonFXSimulation rollerMotorLeft;

    private final TalonFXSimulation rollerMotorRight;

    private final LoggedMechanism2d intakeMechanism;
    private final LoggedMechanismRoot2d intakePivot;
    private final LoggedMechanismLigament2d intakeArm;

    private boolean hopperExpanded = false;

    public IntakeIOSim() {
        this(getPivotMotor(IntakeConstants.IntakeDeviceIds.INTAKE_PIVOT_MOTOR), 
            getRollerMotor(IntakeConstants.IntakeDeviceIds.INTAKE_ROLLER_MOTOR_LEFT), 
            getRollerMotor(IntakeConstants.IntakeDeviceIds.INTAKE_ROLLER_MOTOR_RIGHT));
    }

    private IntakeIOSim(TalonFXSimulation pivotMotor, TalonFXSimulation rollerMotorLeft, TalonFXSimulation rollerMotorRight) {
        super(pivotMotor, rollerMotorLeft, rollerMotorRight);
        this.pivotMotor = pivotMotor;
        this.rollerMotorLeft = rollerMotorLeft;
        this.rollerMotorRight = rollerMotorRight;

        rollerMotorLeft.linkToReference(rollerMotorRight);

        this.intakeMechanism = new LoggedMechanism2d(1.0, 1.0, new Color8Bit(36, 36, 36));
        this.intakePivot = intakeMechanism.getRoot("Intake Pivot", 0.5, 0.3);
        this.intakeArm = intakePivot.append(
            new LoggedMechanismLigament2d(
                "Intake Arm", 
                IntakeSettings.Pivot.PIVOT_ARM_LENGTH, 
                IntakeSettings.Pivot.INITIAL_ANGLE, 
                6.0, 
                new Color8Bit(0, 180, 255)
            )
        );
    }

    private void updateMechanism(IntakeIOInputs inputs) {
        Angle pivotAngle = inputs.pivotMotorInputs.position;
        intakeArm.setAngle(pivotAngle);
        Logger.recordOutput("Intake/Mechanism", intakeMechanism);
        
        Pose3d intakePose = new Pose3d(
            IntakeSimulationSettings.PIVOT_OFFSET.getTranslation(),
            IntakeSimulationSettings.PIVOT_OFFSET.getRotation().plus(
                new Rotation3d(
                    0,
                    pivotAngle.in(Radians),
                    0
                )
            )
        );
        Logger.recordOutput("Intake/MechanismPose", intakePose);

        double armEndX = IntakeSimulationSettings.PIVOT_END_X
            + IntakeSettings.Pivot.PIVOT_ARM_LENGTH.in(Meters)
                * Math.sin(pivotAngle.in(Radians));

        Transform3d hopperOffset = IntakeSimulationSettings.HOPPER_OFFSET;

        double hopperX;
        if (hopperExpanded) {
            hopperX = IntakeSimulationSettings.EXPANDED_HOPPER_X;
        } else {
            hopperX = Math.max(
                IntakeSimulationSettings.MIN_HOPPER_X,
                armEndX + hopperOffset.getX()
            );
        }

        Pose3d hopperPose = new Pose3d(
            hopperX,
            hopperOffset.getY(),
            hopperOffset.getZ(),
            hopperOffset.getRotation()
        );

        Logger.recordOutput("Hopper/MechanismPose", hopperPose);

        if (!hopperExpanded) {
            if (Math.abs(hopperX - IntakeSimulationSettings.EXPANDED_HOPPER_X) <= IntakeSimulationSettings.EXPANDED_TOLERANCE) {
                hopperExpanded = true;
            }
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        pivotSim.update(Settings.DT);
        pivotMotor.refresh();
        rollerSim.update(Settings.DT);
        rollerMotorLeft.refresh();
        rollerMotorRight.refresh();
        super.updateInputs(inputs);

        updateMechanism(inputs);
    }
}