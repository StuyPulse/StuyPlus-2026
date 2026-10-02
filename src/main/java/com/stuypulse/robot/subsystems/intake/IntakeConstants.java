package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public interface IntakeConstants {
    public interface IntakeSettings{
        public interface Pivot {

            // state angles
            // TODO:Get new pivot angles
            Angle INITIAL_ANGLE = Degrees.of(-102);

            Angle STOW_ANGLE = Degrees.of(-102);

            Angle DEPLOY_ANGLE = Degrees.of(-22);

            Angle AGITATE_UP_ANGLE = Degrees.of(-62);

            Angle DIGEST_ANGLE = Degrees.of(-92);

            Angle AGITATE_DOWN_ANGLE = Degrees.of(-22);

            // misc
            Angle ANGLE_TOLERANCE = Degrees.of(0.5);

            Angle PUSHDOWN_THRESHOLD = Degrees.of(-30);

            DoubleSubscriber PUSHDOWN_CURRENT = DogLog.tunable("Intake/Pivot/Pushdown Current Tuning Amps", 13.0);

            // amps
            Current STALL_CURRENT = Amps.of(25);

            // TODO: set this up?
            Time STALL_DEBOUNCE_SEC = Seconds.of(0.0);

            Voltage HOMING_DOWN_VOLTAGE = Volts.of(3);

            // sysid
            Velocity<VoltageUnit> RAMP_RATE = Volts.of(1).per(Second);

            Voltage STEP_VOLTAGE = Volts.of(1);

            // sim
            Angle MIN_ANGLE = Degrees.of(0);

            Angle MAX_ANGLE = Degrees.of(-102.0);

            double GEAR_RATIO = 60.0;

            Distance PIVOT_ARM_LENGTH = Meters.of(0.1439822);

            // mass in kg
            MomentOfInertia MOI = KilogramSquareMeters.of(SingleJointedArmSim.estimateMOI(PIVOT_ARM_LENGTH.in(Meters), 1));
        }

        public interface Roller {

            Current STALL_CURRENT = Amps.of(50);

            Time STALL_DEBOUNCE_SEC = Seconds.of(0.1);

            double GEAR_RATIO = 16.0 / 27.0;

            MomentOfInertia J = KilogramSquareMeters.of(0.001);

            double IDLE_DUTY_CYCLE = 0;

            double INTAKE_DUTY_CYCLE = 1;

            double OUTTAKE_DUTY_CYCLE = -1;
        }
    }

        public interface IntakeGains {

        // pivot gains
        double kP = 300;//300

        double kI = 0;

        double kD = 75;

        Current kS = Amps.of(0);

        Current kV = Amps.of(0);

        Current kA = Amps.of(0);

        Current kG = Amps.of(-12);

        public interface Digestion {

            double kP = 325;

            double kI = 0;

            // TODO: tune
            double kD = 75;
        }
    }

    public interface IntakeDeviceIds{
        int PIVOT_LIMIT_SWITCH = 0;

        int INTAKE_ROLLER_MOTOR_LEFT = 22;

        int INTAKE_ROLLER_MOTOR_RIGHT = 17;

        int INTAKE_PIVOT_MOTOR = 10;
    }

    public interface IntakeMotorConfigs{
		TalonFXConfig PIVOT_CONFIG = new TalonFXConfig()
				.withSupplyCurrentLimitAmps(30)
				.withStatorCurrentLimitAmps(40)
				.withInvertedValue( // not necessarily true, get inverted val
						InvertedValue.Clockwise_Positive)
				.withNeutralMode(NeutralModeValue.Brake)
				.withSensorToMechanismRatio(IntakeConstants.IntakeSettings.Pivot.GEAR_RATIO)
				.withGravityType(GravityTypeValue.Arm_Cosine)
				.withPIDConstants(IntakeConstants.IntakeGains.kP, IntakeConstants.IntakeGains.kI, IntakeConstants.IntakeGains.kD, 0)
				.withFFConstants(
						IntakeConstants.IntakeGains.kS.in(Amps),
						IntakeConstants.IntakeGains.kA.in(Amps),
						IntakeConstants.IntakeGains.kV.in(Amps),
						IntakeConstants.IntakeGains.kG.in(Amps), // regular constants
						0)
				.withPIDConstants(
						IntakeConstants.IntakeGains.Digestion.kP, IntakeConstants.IntakeGains.Digestion.kI, IntakeConstants.IntakeGains.Digestion.kD, 1)
				.withFFConstants(
						IntakeConstants.IntakeGains.kS.in(Amps),
						IntakeConstants.IntakeGains.kA.in(Amps),
						IntakeConstants.IntakeGains.kV.in(Amps),
						IntakeConstants.IntakeGains.kG.in(Amps), // digestion constants
						1);

		TalonFXConfig LEFT_ROLLER_CONFIG = // TODO: apply later
				new TalonFXConfig()
						.withStatorCurrentLimitAmps(50)
						.withInvertedValue( // not necessarily true, get inverted val
								InvertedValue.CounterClockwise_Positive)
						.withNeutralMode(NeutralModeValue.Coast)
						.withSensorToMechanismRatio(IntakeConstants.IntakeSettings.Roller.GEAR_RATIO);

		TalonFXConfig RIGHT_ROLLER_CONFIG = // TODO: apply later
				new TalonFXConfig()
						.withStatorCurrentLimitAmps(50)
						.withInvertedValue(InvertedValue.Clockwise_Positive)
						.withNeutralMode(NeutralModeValue.Coast)
						.withSensorToMechanismRatio(IntakeConstants.IntakeSettings.Roller.GEAR_RATIO);
	}
}
