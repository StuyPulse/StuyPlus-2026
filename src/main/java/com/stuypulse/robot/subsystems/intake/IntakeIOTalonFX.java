/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public class IntakeIOTalonFX extends IntakeIOTalonFXBase {
    public IntakeIOTalonFX() {
        super(new LoggedTalonFX(IntakeConstants.IntakeDeviceIds.INTAKE_PIVOT_MOTOR), 
        new LoggedTalonFX(IntakeConstants.IntakeDeviceIds.INTAKE_ROLLER_MOTOR_LEFT), 
        new LoggedTalonFX(IntakeConstants.IntakeDeviceIds.INTAKE_ROLLER_MOTOR_RIGHT));
    }
}