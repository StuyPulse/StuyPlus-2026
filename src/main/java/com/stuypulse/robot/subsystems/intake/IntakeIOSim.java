/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim extends IntakeIOTalonFXBase {
    private static TalonFXSimulation getPivotMotor(int id) {
        final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        Settings.Intake.Pivot.MOI.in(KilogramSquareMeters),
                        Settings.Intake.Pivot.GEAR_RATIO),
                DCMotor.getKrakenX60(1),
                Settings.Intake.Pivot.GEAR_RATIO,
                Settings.Intake.Pivot.PIVOT_ARM_LENGTH.in(Meters),
                Settings.Intake.Pivot.MAX_ANGLE.in(Radians),
                Settings.Intake.Pivot.MIN_ANGLE.in(Radians), // reversed because negative?
                true,
                Settings.Intake.Pivot.INITIAL_ANGLE.in(Radians));
        final TalonFXSimulation pivotMotor = new TalonFXSimulation(id, pivotSim);
        return pivotMotor;
    }
    
    private static final DCMotorSim rollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2),
            Settings.Intake.Roller.J.in(KilogramSquareMeters),
            Settings.Intake.Roller.GEAR_RATIO),
        DCMotor.getKrakenX60(2));
    private static TalonFXSimulation getRollerMotor(int id) {
        final TalonFXSimulation rollerMotor = new TalonFXSimulation(id, rollerSim);
        return rollerMotor;
    }

    private final TalonFXSimulation pivotMotor;

    private final TalonFXSimulation rollerMotorLeft;

    private final TalonFXSimulation rollerMotorRight;

    public IntakeIOSim() {
        this(getPivotMotor(Ports.Intake.INTAKE_PIVOT_MOTOR), getRollerMotor(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT), getRollerMotor(Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT));
    }

    private IntakeIOSim(TalonFXSimulation pivotMotor, TalonFXSimulation rollerMotorLeft, TalonFXSimulation rollerMotorRight) {
        super(pivotMotor, rollerMotorLeft, rollerMotorRight);
        this.pivotMotor = pivotMotor;
        this.rollerMotorLeft = rollerMotorLeft;
        this.rollerMotorRight = rollerMotorRight;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rollerMotorLeft.update(Settings.DT);
        rollerMotorRight.update(Settings.DT);
        pivotMotor.update(Settings.DT);
        super.updateInputs(inputs);
    }
}