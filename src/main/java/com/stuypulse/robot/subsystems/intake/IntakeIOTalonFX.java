/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public class IntakeIOTalonFX extends IntakeIOTalonFXBase {
    private static LoggedTalonFX getPivotMotor(int id) {
        final LoggedTalonFX pivotMotor = new LoggedTalonFX(id, Settings.CANBUS);
        return pivotMotor;
    }

    private static LoggedTalonFX getRollerMotor(int id) {
        final LoggedTalonFX rollerMotor = new LoggedTalonFX(id, Settings.CANBUS);
        rollerMotor.withSignal(rollerMotor.getDutyCycle());
        return rollerMotor;
    }

    public IntakeIOTalonFX() {
        super(getPivotMotor(Ports.Intake.INTAKE_PIVOT_MOTOR), getRollerMotor(Ports.Intake.INTAKE_ROLLER_MOTOR_LEFT), getRollerMotor(Ports.Intake.INTAKE_ROLLER_MOTOR_RIGHT));
    }
}