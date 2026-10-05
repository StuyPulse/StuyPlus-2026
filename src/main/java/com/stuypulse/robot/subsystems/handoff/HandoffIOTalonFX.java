/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;

public class HandoffIOTalonFX extends HandoffIOTalonFXBase {
    public HandoffIOTalonFX() {
        super(new TalonFX(HandoffConstants.HandoffDeviceIds.HANDOFF_MOTOR));
    }
}