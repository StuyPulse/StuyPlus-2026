/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;

public abstract class // handoff is feeder ---> shooter btw
Handoff extends // handoff is feeder ---> shooter btw
SubsystemBase {

    private static final Handoff instance;

    private HandoffState state;

    static {
        if (Robot.isReal()) {
            instance = new HandoffImpl();
        } else {
            instance = new HandoffSim();
        }
    }

    public static Handoff getInstance() {
        return instance;
    }

    protected Handoff() {
        this.state = HandoffState.IDLE;
    }

    public void setState(HandoffState state) {
        this.state = state;
    }

    public HandoffState getState() {
        return this.state;
    }

    public enum HandoffState {

        IDLE(Settings.Handoff.IDLE_DUTY_CYCLE), FORWARD(Settings.Handoff.FORWARD_DUTY_CYCLE), REVERSE(Settings.Handoff.REVERSE_DUTY_CYCLE);

        private double handoffMotorDutyCycle;

        private HandoffState(double handoffMotorDutyCycle) {
            this.handoffMotorDutyCycle = handoffMotorDutyCycle;
        }

        public double getHandoffDutyCycle() {
            return handoffMotorDutyCycle;
        }
    }

    protected abstract void stopMotors();

    @Override
    public void periodic() {
        final HandoffState currentState = getState();
        DogLog.log("Handoff/State", currentState.name());
        DogLog.log("States/Handoff", currentState.name());
        DogLog.log("Shooter/Handoff Target Duty Cycle", currentState.getHandoffDutyCycle());
    }
}
