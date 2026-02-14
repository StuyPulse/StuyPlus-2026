package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase{
    private static final Feeder instance;
    private FeederState state;

    static {
        if (Robot.isReal()) {
            instance = new FeederImpl();
        } else {
            instance = new FeederSim();
        }
    }

    public static Feeder getInstance(){
        return instance;
    }

    public enum FeederState {
        STOP(0),
        REVERSE(Settings.Feeder.FEEDER_REVERSE),
        FORWARD(Settings.Feeder.FEEDER_FORWARD);

        private double targetRPM;

        private FeederState(double targetRPM){
            this.targetRPM = targetRPM;
        }

        public double getTargetRPM(){
            return this.targetRPM;
        }
    }
    
    protected Feeder() {
        this.state = FeederState.STOP;
    }

    public void setState(FeederState state){
        this.state = state;
    }

    public FeederState getState() {
        return state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Feeder/State", getState().name());
        SmartDashboard.putString("States/Feeder", getState().name());
    }
}