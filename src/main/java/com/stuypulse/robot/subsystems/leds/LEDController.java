/**
 * ********************** PROJECT RON ************************
 */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/**
 * ***********************************************************
 */
package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {

    private final static LEDController instance;

    private final LEDPattern defaultPattern = LEDPattern.kOff;

    private AddressableLED led;

    private AddressableLEDBuffer buffer;

    private AddressableLEDBufferView shooterView;

    private AddressableLEDBufferView feederView;

    private AddressableLEDBufferView intakeView;

    private AddressableLEDBufferView handoffView;

    static {
        instance = new LEDController();
    }

    public static LEDController getInstance() {
        return instance;
    }

    protected LEDController() {
        this.led = new AddressableLED(Ports.LED.LED_PWM_PORT);
        this.buffer = new AddressableLEDBuffer(Settings.LED.LED_LENGTH);
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
        this.shooterView = buffer.createView(Settings.LED.SHOOTER_BUFFER[0], Settings.LED.SHOOTER_BUFFER[1]);
        this.feederView = buffer.createView(Settings.LED.FEEDER_BUFFER[0], Settings.LED.FEEDER_BUFFER[1]);
        this.intakeView = buffer.createView(Settings.LED.INTAKE_BUFFER[0], Settings.LED.INTAKE_BUFFER[1]);
        this.handoffView = buffer.createView(Settings.LED.HANDOFF_BUFFER[0], Settings.LED.HANDOFF_BUFFER[1]);
        applyPattern(defaultPattern);
        SmartDashboard.putData(instance);
    }

    public void applyShoot(LEDPattern pattern) {
        pattern.applyTo(shooterView);
    }

    public void applyFeed(LEDPattern pattern) {
        pattern.applyTo(feederView);
    }

    public void applyIntake(LEDPattern pattern) {
        pattern.applyTo(intakeView);
    }

    public void applyHandoff(LEDPattern pattern) {
        pattern.applyTo(handoffView);
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    @Override
    public void periodic() {
        if (!Settings.EnabledSubsystems.LED.get()) {
            LEDPattern.kOff.applyTo(buffer);
        }
        led.setData(buffer);
    }
}
