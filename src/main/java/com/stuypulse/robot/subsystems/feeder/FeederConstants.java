package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public interface FeederConstants {
    public interface FeederSettings{
        Voltage REVERSE_VOLTAGE = Volts.of(-10.0); // TODO: get
        DoubleSubscriber REVERSE_TIME_BEFORE_SHOOT = DogLog.tunable("Feeder/Seconds To Reverse Before Shooting", 0.75);

        Voltage FORWARD_VOLTAGE = Volts.of(10.0);

        // TODO: get from mec
        double GEAR_RATIO = 34/14; // (34/14) : 1

        MomentOfInertia J = KilogramSquareMeters.of(0.001);
    }

    public interface FeederDeviceIds {

        int FEEDER_MOTOR = 15;
    }

    public interface FeederMotorConfigs{
        // TODO: get values after motor pinion swap
		TalonFXConfig LEADER_CONFIG = new TalonFXConfig()
			.withStatorCurrentLimitAmps(80)
			.withNeutralMode(NeutralModeValue.Coast)
			.withInvertedValue(InvertedValue.Clockwise_Positive)
			.withSensorToMechanismRatio(FeederConstants.FeederSettings.GEAR_RATIO);
    }
}
