package com.stuypulse.robot.subsystems.shifttimer;

import java.util.Optional;

import com.stuypulse.robot.constants.Shifts;
import com.stuypulse.robot.constants.Shifts.ShiftState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShiftTimer extends SubsystemBase {
    private ShiftState currentState;
    private Optional<Alliance> alliance;
    private boolean isFirstActiveAlliance;
    private boolean isHubActive;
    private double matchTime;

    private final static ShiftTimer instance;

    static {
        instance = new ShiftTimer();
    }

    public static ShiftTimer getInstance() {
        return instance;
    }

    protected ShiftTimer() {
        this.alliance = DriverStation.getAlliance();
        if (this.alliance.isEmpty()) {
            this.currentState = ShiftState.NOT_GAME; // if there is no alliance, this means that the robot is likely not
                                                     // in competition
        }

    }

    private String formatTime(double time) {
        int minutes = (int) Math.floor(time / 60);
        int seconds = (int) (time % 60);

        return (minutes < 10 ? "0" + minutes : minutes) + ":" + (seconds < 10 ? "0" + seconds : seconds);
    }

    // Source:
    // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
    // Modified to fit the subsystem
    public void setIsHubActive() {
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            this.isHubActive = false;
            return;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            this.isHubActive = true;
            return;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            this.isHubActive = false;
            return;
        }

        // We're teleop enabled, compute.
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            this.isHubActive = true;
            return;
        }

        switch (gameData.charAt(0)) {
            case 'R' -> this.isFirstActiveAlliance = alliance.get() == Alliance.Red;
            case 'B' -> this.isFirstActiveAlliance = alliance.get() == Alliance.Blue;
            default -> {
                // If we have invalid game data, assume hub is active.
                this.isHubActive = true;
                return;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !isFirstActiveAlliance;
            case Blue -> isFirstActiveAlliance;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            this.isHubActive = true;
            return;
        } else if (matchTime > 105) {
            // Shift 1
            this.isHubActive = shift1Active;
            return;
        } else if (matchTime > 80) {
            // Shift 2
            this.isHubActive = !shift1Active;
            return;
        } else if (matchTime > 55) {
            // Shift 3
            this.isHubActive = shift1Active;
            return;
        } else if (matchTime > 30) {
            // Shift 4
            this.isHubActive = !shift1Active;
            return;
        } else {
            // End game, hub always active.
            this.isHubActive = true;
            return;
        }
    }

    public boolean isHubActive() {
        return this.isHubActive;
    }

    public enum RobotState {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST
    }

    public RobotState getRobotMode() {
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

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        if (this.currentState != ShiftState.NOT_GAME) {
            setIsHubActive();
        }

        SmartDashboard.putString("ShiftTimer/ShiftState", this.currentState.name());
        SmartDashboard.putBoolean("ShiftTimer/IsHubActive", this.isHubActive);
    }
}
