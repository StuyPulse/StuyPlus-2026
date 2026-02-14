package com.stuypulse.robot.subsystems.led;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private final static LEDController instance;

    private final LEDPattern defaultPattern = LEDPattern.kOff;
    private AddressableLED led;
    private AddressableLEDSim ledSim;
    private AddressableLEDBuffer buffer;

    static {
        instance = new LEDController(Ports.LED.PORT, Settings.LED.LED_LENGTH);
    }

    public static LEDController getInstance() {
        return instance;
    }

    protected LEDController(int port, int length) {
        this.led = new AddressableLED(port);
        this.ledSim = new AddressableLEDSim(led);
        this.buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();

        applyPattern(defaultPattern);

        SmartDashboard.putData(instance);
    } 

    public void applyPattern(LEDPattern pattern) {
        if (!Settings.EnabledSubsystems.LED.get()) return; // cuz 20 milliseconds intervals, it might blink, and it's unnecessary to set it if disabled
        pattern.applyTo(buffer);
    }

    @Override
    public void periodic() {
        if (Settings.EnabledSubsystems.LED.get()) {
            led.start();
            led.setData(buffer);
        } else {
            LEDPattern.kOff.applyTo(buffer);
            led.setData(buffer);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (ledSim.getInitialized()) {
            // Logger.processInputs
        }
    }
} 