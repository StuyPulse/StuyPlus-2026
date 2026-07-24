/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

public class FeederIOTalonFX extends FeederIOTalonFXBase {
    private static TalonFX getFeederMotor() {
        TalonFX motor = new TalonFX(Ports.Feeder.FEEDER_MOTOR, Settings.CANBUS);
        Motors.Feeder.LEADER_CONFIG.configure(motor);
        return motor;
    }

    public FeederIOTalonFX() {
        super(getFeederMotor());
    }
}