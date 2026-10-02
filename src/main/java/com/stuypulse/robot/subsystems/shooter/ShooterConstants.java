package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterConstants {

    public interface ShooterSettings {
        DoubleSubscriber FIRST_SHOT_BONUS = DogLog.tunable("Shooter/First_shot_bonus_RPM", 250.0);
        Time FIRST_SHOT_DEBOUNCE = Seconds.of(6.7);

        Time SHOOT_TIME_AUTO = Seconds.of(1.5);

        Velocity<VoltageUnit> RAMP_RATE = Volts.of(1).per(Second);

        Voltage STEP_VOLTAGE = Volts.of(7);

        Distance WHEEL_RADIUS = Inches.of(4);

        // Sim
        MomentOfInertia J = KilogramSquareMeters.of(0.1);

        double GEAR_RATIO = 0.1;

        // TODO: get
        Distance FLYWHEEL_RADIUS = Inches.of(3);

        // TODO: Test for manual shooting RPM
        DoubleSubscriber MANUAL_HUB_RPM = DogLog.tunable("Shooter/Manual Shot Tuning RPM", 3650.0);

        AngularVelocity MIN_SHOOTER_VELOCITY = RPM.of(1740);

        DoubleSubscriber SHOOT_TUNING_RPM = DogLog.tunable("Shooter/Shoot Tuning RPM", 0.0);
        DoubleSubscriber FERRY_TUNING_RPM = DogLog.tunable("Shooter/Ferry Tuning RPM", 0.0);

        AngularVelocity SHOOTER_SPUN_UP_TOLERANCE = RPM.of(100);
        public interface RPMInterpolation {

            double[][] distanceRPMInterpolationValues = {
                {1.46, 2600},
                {2.07, 3150},
                {3.13, 3700},
                {3.45, 3933},
                {4.13, 4200}
                //TODO: These numbers don't make sense
                // { 4.895367348608047, 3250.0 },
                // { 6.1322461808798705, 3487.0 } 
            };
        }

        // These values are placeholders and should be replaced with actual data from testing
        public interface TOFInterpolation {

            double[][] distanceTOFInterpolationValues = {
                { 1.0, 0.5 },
                { 2.0, 0.75 },
                { 3.0, 1.0 },
                { 4.0, 1.25 },
                { 5.0, 1.5 } };
        }

        // These values are placeholders and should be replaced with actual data from testing
        public interface FerryRPMInterpolation {

            double[][] ferryDistanceRPMInterpolation = {
                { 1.0, 2300.0 },
                { 2.0, 2800.0 },
                { 3.0, 3300.0 },
                { 4.0, 3800.0 },
                { 5.0, 5500.0 } };
        }

        // These values are placeholders and should be replaced with actual data from testing
        public interface FerryTOFInterpolation {

            double[][] FerryTOFInterpolationInterpolation = {
                { 1.0, 0.5 },
                { 2.0, 0.75 },
                { 3.0, 1.0 },
                { 4.0, 1.25 },
                { 5.0, 1.5 } };
        }
        // These values are placeholders and should be replaced with actual data from testing
    }

    public interface ShooterGains {
        DoubleSubscriber kP = DogLog.tunable("Shooter/kP", 15.0);

        DoubleSubscriber kI = DogLog.tunable("Shooter/kI", 0.0);

        DoubleSubscriber kD = DogLog.tunable("Shooter/kD", 0.0);

        DoubleSubscriber kS = DogLog.tunable("Shooter/kS", 2.5);

        DoubleSubscriber kV = DogLog.tunable("Shooter/kV", 0.05);

        DoubleSubscriber kA = DogLog.tunable("Shooter/kA", 0.0);

        public interface FirstShot {
            double kP = 20;
            double kI = 0;
            double kD = 0;
            }
        }

    public interface ShooterDeviceIds {
        int SHOOTER_MOTOR_LEFT = 30;

        // TODO: get after champs
        int SHOOTER_MOTOR_CENTER = 54;

        int SHOOTER_MOTOR_RIGHT = 47;
    }

	public interface ShooterMotorConfigs {
		TalonFXConfig SHOOTER_MOTOR_LEFT = new TalonFXConfig()
				.withPIDConstants(ShooterConstants.ShooterGains.kP.get(), ShooterConstants.ShooterGains.kI.get(), ShooterConstants.ShooterGains.kD.get(), 0)
				.withPIDConstants(ShooterConstants.ShooterGains.FirstShot.kP, ShooterConstants.ShooterGains.FirstShot.kI, ShooterConstants.ShooterGains.FirstShot.kD, 1)
				.withSupplyCurrentLimitAmps(200)
				.withStatorCurrentLimitAmps(200)
				.withNeutralMode(NeutralModeValue.Coast)
				.withFFConstants(ShooterConstants.ShooterGains.kS.get(), ShooterConstants.ShooterGains.kV.get(), ShooterConstants.ShooterGains.kA.get(), 0)
				.withInvertedValue(InvertedValue.CounterClockwise_Positive);

		TalonFXConfig SHOOTER_MOTOR_CENTER = new TalonFXConfig()
				.withPIDConstants(ShooterConstants.ShooterGains.kP.get(), ShooterConstants.ShooterGains.kI.get(), ShooterConstants.ShooterGains.kD.get(), 0)
				.withPIDConstants(ShooterConstants.ShooterGains.FirstShot.kP, ShooterConstants.ShooterGains.FirstShot.kI, ShooterConstants.ShooterGains.FirstShot.kD, 1)
				.withSupplyCurrentLimitAmps(200)
				.withStatorCurrentLimitAmps(200)
				.withNeutralMode(NeutralModeValue.Coast)
				.withFFConstants(ShooterConstants.ShooterGains.kS.get(), ShooterConstants.ShooterGains.kV.get(), ShooterConstants.ShooterGains.kA.get(), 0)
				.withInvertedValue(InvertedValue.CounterClockwise_Positive);

		TalonFXConfig SHOOTER_MOTOR_RIGHT = new TalonFXConfig()
				.withPIDConstants(ShooterConstants.ShooterGains.kP.get(), ShooterConstants.ShooterGains.kI.get(), ShooterConstants.ShooterGains.kD.get(), 0)
				.withPIDConstants(ShooterConstants.ShooterGains.FirstShot.kP, ShooterConstants.ShooterGains.FirstShot.kI, ShooterConstants.ShooterGains.FirstShot.kD, 1)
				.withSupplyCurrentLimitAmps(200)
				.withStatorCurrentLimitAmps(200)
				.withNeutralMode(NeutralModeValue.Coast)
				.withFFConstants(ShooterConstants.ShooterGains.kS.get(), ShooterConstants.ShooterGains.kV.get(), ShooterConstants.ShooterGains.kA.get(), 0)
				.withFFConstants(ShooterConstants.ShooterGains.kS.get(), ShooterConstants.ShooterGains.kV.get(), ShooterConstants.ShooterGains.kA.get(), 1)
				.withInvertedValue(InvertedValue.Clockwise_Positive);
	}
}