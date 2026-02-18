package com.stuypulse.robot.subsystems.shifttimer;

import java.util.function.Supplier;

import com.stuypulse.robot.RobotContainer.RobotMode;
import com.stuypulse.robot.constants.Shifts;
import com.stuypulse.robot.constants.Shifts.ShiftState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShiftTimer extends SubsystemBase {
    private Timer timer;
    private ShiftState currentState;
    private Supplier<RobotMode> robotModeSupplier;

    public ShiftTimer(Supplier<RobotMode> supplier) {
        this.timer = new Timer();

        this.robotModeSupplier = supplier;
        
        if (!isCompetition()) {
            this.currentState = ShiftState.NOT_GAME;
        } else {
            this.currentState = ShiftState.PRE_GAME;
        }
    }

    private boolean isCompetition() {
        return DriverStation.isFMSAttached();
        // return true; // for testing purposes
    }

    private String formatTime(double time) {
        int minutes = (int) Math.floor(time / 60);
        int seconds = (int) (time % 60);

        return (minutes < 10 ? "0" + minutes : minutes) + ":" + (seconds < 10 ? "0" + seconds : seconds);
    }

    @Override
    public void periodic() {
        if (this.currentState == ShiftState.NOT_GAME) {
            SmartDashboard.putString("ShiftTimer/ShiftState", ShiftState.NOT_GAME.name());
            return;
        }

        if (this.robotModeSupplier.get() == RobotMode.AUTONOMOUS) {
            if (this.currentState == ShiftState.PRE_GAME) {
                timer.start();
                this.currentState = ShiftState.AUTO;
            } else {
                if (timer.get() >= Shifts.AUTO) {
                    timer.stop();
                    timer.reset();
                }
            }
        }

        if (this.robotModeSupplier.get() == RobotMode.TELEOP) {
            if (this.currentState == ShiftState.AUTO) {
                timer.start();
                this.currentState = ShiftState.TRANSITION_SHIFT;
            }
            if (timer.get() >= Shifts.TRANSITION_SHIFT) {
                this.currentState = ShiftState.SHIFT_1;
            }
            if (timer.get() >= Shifts.SHIFT_1) {
                this.currentState = ShiftState.SHIFT_2;
            }
            if (timer.get() >= Shifts.SHIFT_2) {
                this.currentState = ShiftState.SHIFT_3;
            }
            if (timer.get() >= Shifts.SHIFT_3) {
                this.currentState = ShiftState.SHIFT_4;
            }
            if (timer.get() >= Shifts.SHIFT_4) {
                this.currentState = ShiftState.END_GAME;
            }
            if (timer.get() >= Shifts.END_GAME) {
                this.currentState = ShiftState.POST_GAME;
                timer.stop();
                timer.reset();
            }
        }

        SmartDashboard.putString("ShiftTimer/ShiftState", this.currentState.name());
        SmartDashboard.putString("ShiftTimer/RobotState", this.robotModeSupplier.get().name());
        SmartDashboard.putString("ShiftTimer/Time", formatTime(timer.get()));
        SmartDashboard.putNumber("ShiftTimer/RawTimer", timer.get());
    }
}
