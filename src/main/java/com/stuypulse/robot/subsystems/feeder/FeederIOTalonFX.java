/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

public class FeederIOTalonFX extends FeederIOTalonFXBase {
    public FeederIOTalonFX() {
        super(new TalonFX(FeederConstants.FeederDeviceIds.FEEDER_MOTOR));
    }
}