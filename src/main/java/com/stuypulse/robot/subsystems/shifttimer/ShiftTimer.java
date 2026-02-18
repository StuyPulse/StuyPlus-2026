package com.stuypulse.robot.subsystems.shifttimer;

import java.sql.Driver;
import java.util.Optional;
import java.util.function.Supplier;

import com.stuypulse.robot.constants.Shifts;
import com.stuypulse.robot.constants.Shifts.ShiftState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShiftTimer extends SubsystemBase {
    private Timer timer;
    private ShiftState currentState;
    public String gameData;
    Optional<Alliance> alliance;
    Boolean isHubActive;
    Alliance firstActiveAlliance;

    private final static ShiftTimer instance;

    static {
        instance = new ShiftTimer();
    }

    public static ShiftTimer getInstance() {
        return instance;
    }

    protected ShiftTimer() {
        this.timer = new Timer();

        this.isHubActive = false;

        this.alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            this.currentState = ShiftState.NOT_GAME;
        } else {
            this.currentState = ShiftState.PRE_GAME;
        }
        SmartDashboard.putString("ShiftTimer/ShiftState", this.currentState.name());
    }

    private String formatTime(double time) {
        int minutes = (int) Math.floor(time / 60);
        int seconds = (int) (time % 60);

        return (minutes < 10 ? "0" + minutes : minutes) + ":" + (seconds < 10 ? "0" + seconds : seconds);
    }

    private void setFirstActiveAlliance() {
        this.gameData = DriverStation.getGameSpecificMessage();
        SmartDashboard.putString("ShiftTimer/GameData", gameData);

        switch (gameData.charAt(0)) {
            case 'R' -> {
                this.firstActiveAlliance = Alliance.Blue;
            }
            case 'B' -> {
                this.firstActiveAlliance = Alliance.Red;
            }
        }
    }

    private RobotState getRobotMode() {
        if (DriverStation.isDisabled())
            return RobotState.DISABLED;
        if (DriverStation.isAutonomousEnabled())
            return RobotState.AUTONOMOUS;
        if (DriverStation.isTeleopEnabled())
            return RobotState.TELEOP;
        if (DriverStation.isTestEnabled())
            return RobotState.TEST;
        return null;
    }

    public enum RobotState {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST
    }

    @Override
    public void periodic() {
        if (this.currentState == ShiftState.NOT_GAME)
            return;

        if (getRobotMode() == RobotState.AUTONOMOUS) {
            if (this.currentState == ShiftState.PRE_GAME) {
                timer.start();
                this.currentState = ShiftState.AUTO;
                this.isHubActive = true;
            } else {
                if (timer.get() >= Shifts.AUTO) {
                    timer.stop();
                    timer.reset();
                }
            }
        } else {
            if (this.firstActiveAlliance == null) {
                setFirstActiveAlliance();
                return;
            }
        }

        if (getRobotMode() == RobotState.TELEOP) {
            if (this.currentState == ShiftState.AUTO) {
                timer.start();
                this.currentState = ShiftState.TRANSITION_SHIFT;
                this.isHubActive = true;
            }
            if (timer.get() >= Shifts.TRANSITION_SHIFT) {
                this.currentState = ShiftState.SHIFT_1;
                this.isHubActive = alliance.get() == this.firstActiveAlliance;
            }
            if (timer.get() >= Shifts.SHIFT_1) {
                this.currentState = ShiftState.SHIFT_2;
                this.isHubActive = !this.isHubActive;
            }
            if (timer.get() >= Shifts.SHIFT_2) {
                this.currentState = ShiftState.SHIFT_3;
                this.isHubActive = !this.isHubActive;
            }
            if (timer.get() >= Shifts.SHIFT_3) {
                this.currentState = ShiftState.SHIFT_4;
                this.isHubActive = !this.isHubActive;
            }
            if (timer.get() >= Shifts.SHIFT_4) {
                this.isHubActive = true;
                this.currentState = ShiftState.END_GAME;
            }
            if (timer.get() >= Shifts.END_GAME) {
                this.currentState = ShiftState.POST_GAME;
                timer.stop();
            }
        }

        SmartDashboard.putString("ShiftTimer/ShiftState", this.currentState.name());
        SmartDashboard.putString("ShiftTimer/RobotState", getRobotMode().name());
        SmartDashboard.putString("ShiftTimer/Time", formatTime(timer.get()));
        SmartDashboard.putNumber("ShiftTimer/RawTimer", timer.get());
        SmartDashboard.putBoolean("ShiftTimer/IsHubActive", this.isHubActive);
    }
}
