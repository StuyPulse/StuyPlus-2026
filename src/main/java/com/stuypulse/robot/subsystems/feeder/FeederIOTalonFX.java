/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Settings;

public class FeederIOTalonFX extends FeederIOTalonFXBase {
    private static TalonFX getFeederMotor(int id) {
        final TalonFX motor = new TalonFX(id, Settings.CANBUS);
        return motor;
    }

    public FeederIOTalonFX() {
        super(getFeederMotor(FeederConstants.FeederDeviceIds.FEEDER_MOTOR));
    }
}