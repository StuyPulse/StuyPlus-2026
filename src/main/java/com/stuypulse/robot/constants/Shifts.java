package com.stuypulse.robot.constants;

public interface Shifts {
    int AUTO = 20;
    int TRANSITION_SHIFT = 10;
    int SHIFT_1 = 35;
    int SHIFT_2 = 60;
    int SHIFT_3 = 85;
    int SHIFT_4 = 110;
    int END_GAME = 140;

    public enum ShiftState {
        PRE_GAME,
        AUTO,
        TRANSITION_SHIFT,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        END_GAME,
        POST_GAME,
        NOT_GAME,
    }
}