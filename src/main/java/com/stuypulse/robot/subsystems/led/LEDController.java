package com.stuypulse.robot.subsystems.led;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private final static LEDController instance;

    private final LEDPattern defaultPattern = LEDPattern.kOff;
    private AddressableLED led;
    
    private AddressableLEDBuffer buffer;
    private AddressableLEDBufferView shoot;
    private AddressableLEDBufferView feed;
    private AddressableLEDBufferView intake;

    static {
        instance = new LEDController(Ports.LED.PORT, Settings.LED.LED_LENGTH);
    }

    public static LEDController getInstance() {
        return instance;
    }

    protected LEDController(int port, int length) {
        this.led = new AddressableLED(port);
        
        this.buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();

        this.shoot = buffer.createView(0, 19);
        this.feed = buffer.createView(20, 39);
        this.intake = buffer.createView(40, 59);


        applyPattern(defaultPattern);

        SmartDashboard.putData(instance);
    } 


    public void applyShoot(LEDPattern pattern){
        pattern.applyTo(shoot);
    }

    public void applyFeed(LEDPattern pattern){
        pattern.applyTo(feed);
    }
    
    public void applyIntake (LEDPattern pattern){
        pattern.applyTo(intake);
    }
    

    public void applyPattern(LEDPattern pattern) {
        if (!Settings.EnabledSubsystems.LED.get()) return; // because 20 milliseconds intervals, it might blink, and it's unnecessary to set it if disabled
        //pattern.applyTo(buffer);
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
} 