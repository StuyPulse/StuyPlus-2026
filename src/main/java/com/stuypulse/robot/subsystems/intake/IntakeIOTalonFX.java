/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class IntakeIOTalonFX extends IntakeIOTalonFXBase {
    private static TalonFX getPivotMotor(int id) {
        final TalonFX pivotMotor = new TalonFX(id, Settings.CANBUS);
        return pivotMotor;
    }

    private static TalonFX getRollerMotor(int id) {
        final TalonFX rollerMotor = new TalonFX(id, Settings.CANBUS);
        return rollerMotor;
    }

    public IntakeIOTalonFX() {
        super(getPivotMotor(Ports.Intake.INTAKE_PIVOT_MOTOR), getRollerMotor(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT), getRollerMotor(Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT));
    }
}