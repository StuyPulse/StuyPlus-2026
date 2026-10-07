package com.stuypulse.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public interface LEDConstants {
    public interface LEDSettings {
        // TODO: Get actual length of led, along with length of individual sections
        int LED_LENGTH = 80;

        // Buffer Views {Starting Index, Ending Index}
        int[] SHOOTER_BUFFER = { 0, 19 };

        int[] FEEDER_BUFFER = { 20, 39 };

        int[] INTAKE_BUFFER = { 40, 59 };

        int[] HANDOFF_BUFFER = { 60, 79 };

        // shooter
        LEDPattern SHOOTING = LEDPattern.solid(Color.kOrange);

        LEDPattern FERRYING = LEDPattern.solid(Color.kPurple);

        LEDPattern MANUAL = LEDPattern.solid(Color.kPeru);

        // feeder
        LEDPattern FEEDER_FORWARD = LEDPattern.solid(Color.kBlue);

        LEDPattern FEEDER_REVERSE = LEDPattern.solid(Color.kRed);

        // intake
        LEDPattern INTAKING = LEDPattern.solid(Color.kYellow);

        LEDPattern OUTTAKING = LEDPattern.solid(Color.kGreen);

        LEDPattern HOMING_DOWN = LEDPattern.solid(Color.kGainsboro);

        LEDPattern AGITATING = LEDPattern.solid(Color.kCyan);

        // handoff
        LEDPattern HANDOFF_FORWARD = LEDPattern.solid(Color.kDarkOrange);

        // mmm papaya whip
        LEDPattern HANDOFF_REVERSE = LEDPattern.solid(Color.kPapayaWhip);

        // states
        LEDPattern DISABLED = LEDPattern.solid(Color.kGray);
    }

    public interface LEDPorts {

        // TODO: Get actual port
        int LED_PWM_PORT = 0;
    }
}