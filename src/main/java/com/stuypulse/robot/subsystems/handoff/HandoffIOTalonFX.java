/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class HandoffIOTalonFX extends HandoffIOTalonFXBase {
    private static TalonFX getHandoffMotor(int id) {
        final TalonFX motor = new TalonFX(id, Settings.CANBUS);
        return motor;
    }

    public HandoffIOTalonFX() {
        super(getHandoffMotor(Ports.Handoff.HANDOFF_MOTOR));
    }
}