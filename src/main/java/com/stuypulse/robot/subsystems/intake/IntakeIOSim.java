/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.RobotVisualizer;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim extends IntakeIOTalonFXBase {
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
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        pivotSim.update(Settings.DT);
        pivotMotor.refresh();
        rollerSim.update(Settings.DT);
        rollerMotorLeft.refresh();
        rollerMotorRight.refresh();
        super.updateInputs(inputs);

        RobotVisualizer.getInstance().updateIntake(inputs.pivotMotorInputs.position, inputs.leftRollerMotorInputs.velocity);
    }
}