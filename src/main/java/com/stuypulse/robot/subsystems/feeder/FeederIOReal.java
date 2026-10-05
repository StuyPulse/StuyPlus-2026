/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public class FeederIOReal extends FeederIOBase {
    public FeederIOReal() {
        super(new LoggedTalonFX(FeederConstants.FeederDeviceIds.FEEDER_MOTOR));
    }
}