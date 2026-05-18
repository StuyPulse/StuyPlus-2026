/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoubleSubscriber;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {

	/** Classes to store all of the values a motor needs */
	public interface Intake {

		TalonFXConfig PIVOT_CONFIG = new TalonFXConfig()
				.withSupplyCurrentLimitAmps(30)
				.withStatorCurrentLimitAmps(40)
				.withInvertedValue( // not necessarily true, get inverted val
						InvertedValue.Clockwise_Positive)
				.withNeutralMode(NeutralModeValue.Brake)
				.withSensorToMechanismRatio(Settings.Intake.Pivot.GEAR_RATIO)
				.withGravityType(GravityTypeValue.Arm_Cosine)
				.withPIDConstants(Gains.Intake.kP, Gains.Intake.kI, Gains.Intake.kD, 0)
				.withFFConstants(
						Gains.Intake.kS.in(Amps),
						Gains.Intake.kA.in(Amps),
						Gains.Intake.kV.in(Amps),
						Gains.Intake.kG.in(Amps), // regular constants
						0)
				.withPIDConstants(
						Gains.Intake.Digestion.kP, Gains.Intake.Digestion.kI, Gains.Intake.Digestion.kD, 1)
				.withFFConstants(
						Gains.Intake.kS.in(Amps),
						Gains.Intake.kA.in(Amps),
						Gains.Intake.kV.in(Amps),
						Gains.Intake.kG.in(Amps), // digestion constants
						1);

		TalonFXConfig LEFT_ROLLER_CONFIG = // TODO: apply later
				new TalonFXConfig()
						.withStatorCurrentLimitAmps(50)
						.withInvertedValue( // not necessarily true, get inverted val
								InvertedValue.CounterClockwise_Positive)
						.withNeutralMode(NeutralModeValue.Coast)
						.withSensorToMechanismRatio(Settings.Intake.Roller.GEAR_RATIO);

		TalonFXConfig RIGHT_ROLLER_CONFIG = // TODO: apply later
				new TalonFXConfig()
						.withStatorCurrentLimitAmps(50)
						.withInvertedValue(InvertedValue.Clockwise_Positive)
						.withNeutralMode(NeutralModeValue.Coast)
						.withSensorToMechanismRatio(Settings.Intake.Roller.GEAR_RATIO);
	}

	public interface Feeder {

		// TODO: get values after motor pinion swap
		TalonFXConfig LEADER_CONFIG = new TalonFXConfig()
				.withStatorCurrentLimitAmps(80)
				// .withRampRate(0.25)
				.withNeutralMode(NeutralModeValue.Coast)
				.withInvertedValue(InvertedValue.CounterClockwise_Positive);
	}

	public interface Shooter {

		// TalonFXConfig SHOOTER_MOTOR_CONFIG = new TalonFXConfig()
		// .withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
		// .withStatorCurrentLimitAmps(80)
		// .withRampRate(0.25)
		// .withNeutralMode(NeutralModeValue.Coast)
		// .withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
		// .withInvertedValue(InvertedValue.CounterClockwise_Positive);
		// TalonFXConfig SHOOTER_MOTOR_RIGHT = new TalonFXConfig()
		// .withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
		// .withStatorCurrentLimitAmps(80)
		// .withRampRate(0.25)
		// .withNeutralMode(NeutralModeValue.Coast)
		// .withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
		// .withInvertedValue(InvertedValue.Clockwise_Positive);
		TalonFXConfig SHOOTER_MOTOR_LEFT = new TalonFXConfig()
				.withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
				.withStatorCurrentLimitAmps(80)
				// .withRampRate(0.25)
				.withNeutralMode(NeutralModeValue.Coast)
				.withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
				.withInvertedValue(InvertedValue.CounterClockwise_Positive);

		TalonFXConfig SHOOTER_MOTOR_CENTER = new TalonFXConfig()
				.withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
				.withStatorCurrentLimitAmps(80)
				// .withRampRate(0.25)
				.withNeutralMode(NeutralModeValue.Coast)
				.withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
				.withInvertedValue(InvertedValue.CounterClockwise_Positive);

		TalonFXConfig SHOOTER_MOTOR_RIGHT = new TalonFXConfig()
				.withPIDConstants(Gains.Shooter.kP, Gains.Shooter.kI, Gains.Shooter.kD, 0)
				.withStatorCurrentLimitAmps(80)
				// .withRampRate(0.25)
				.withNeutralMode(NeutralModeValue.Coast)
				.withFFConstants(Gains.Shooter.kS, Gains.Shooter.kV, Gains.Shooter.kA, 0)
				.withInvertedValue(InvertedValue.Clockwise_Positive);
	}

	public interface Handoff {

		TalonFXConfig HANDOFF_MOTOR_CONFIG = new TalonFXConfig()
				.withStatorCurrentLimitAmps(80)
				// .withRampRate(0.25)
				.withNeutralMode(NeutralModeValue.Coast)
				.withInvertedValue(InvertedValue.CounterClockwise_Positive);
	}

	public static class TalonFXConfig {

		private final TalonFXConfiguration configuration = new TalonFXConfiguration();

		private final Slot0Configs slot0Configs = new Slot0Configs();

		private final Slot1Configs slot1Configs = new Slot1Configs();

		private final Slot2Configs slot2Configs = new Slot2Configs();

		private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

		private final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();

		private final OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();

		private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

		private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();

		private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

		private final double[] lastKP = new double[3];
		private final double[] lastKI = new double[3];
		private final double[] lastKD = new double[3];
		private final double[] lastKS = new double[3];
		private final double[] lastKV = new double[3];
		private final double[] lastKA = new double[3];

		public void updateGainsConfig(
				TalonFX motor,
				int slot,
				DoubleSubscriber kP,
				DoubleSubscriber kI,
				DoubleSubscriber kD,
				DoubleSubscriber kS,
				DoubleSubscriber kV,
				DoubleSubscriber kA) {
			if (slot != 0 && slot != 1 && slot != 2) {
				return;
			}

			final double currentKP = kP.get();
			final double currentKI = kI.get();
			final double currentKD = kD.get();
			final double currentKS = kS.get();
			final double currentKV = kV.get();
			final double currentKA = kA.get();

			final boolean changed = currentKP != lastKP[slot]
					|| currentKI != lastKI[slot]
					|| currentKD != lastKD[slot]
					|| currentKS != lastKS[slot]
					|| currentKV != lastKV[slot]
					|| currentKA != lastKA[slot];

			if (!changed) {
				return;
			}

			final SlotConfigs gainConfig = new SlotConfigs()
					.withKP(currentKP)
					.withKI(currentKI)
					.withKD(currentKD)
					.withKS(currentKS)
					.withKV(currentKV)
					.withKA(currentKA);

			gainConfig.SlotNumber = slot;

			motor.getConfigurator().apply(gainConfig);

			lastKP[slot] = currentKP;
			lastKI[slot] = currentKI;
			lastKD[slot] = currentKD;
			lastKS[slot] = currentKS;
			lastKV[slot] = currentKV;
			lastKA[slot] = currentKA;

			switch (slot) {
				case 0:
					motor.getConfigurator().refresh(this.getConfiguration().Slot0);
					break;
				case 1:
					motor.getConfigurator().refresh(this.getConfiguration().Slot1);
					break;
				case 2:
					motor.getConfigurator().refresh(this.getConfiguration().Slot2);
					break;
			}
		}

		public TalonFXConfiguration getConfiguration() {
			return this.configuration;
		}

		public void configure(TalonFX motor) {
			motor.getConfigurator().apply(configuration);
		}

		// SLOT 0 CONFIGS
		public TalonFXConfig withPIDConstants(double kP, double kI, double kD, int slot) {
			switch (slot) {
				case 0:
					configuration.withSlot0(slot0Configs.withKP(kP).withKI(kI).withKD(kD));
					break;
				case 1:
					configuration.withSlot1(slot1Configs.withKP(kP).withKI(kI).withKD(kD));
					break;
				case 2:
					configuration.withSlot2(slot2Configs.withKP(kP).withKI(kI).withKD(kD));
					break;
			}
			return this;
		}

		public TalonFXConfig withFFConstants(double kS, double kV, double kA, int slot) {
			return withFFConstants(kS, kV, kA, 0, slot);
		}

		public TalonFXConfig withFFConstants(double kS, double kV, double kA, double kG, int slot) {
			switch (slot) {
				case 0:
					configuration.withSlot0(slot0Configs.withKS(kS).withKV(kV).withKA(kA).withKG(kG));
					break;
				case 1:
					configuration.withSlot1(slot1Configs.withKS(kS).withKV(kV).withKA(kA).withKG(kG));
					break;
				case 2:
					configuration.withSlot2(slot2Configs.withKS(kS).withKV(kV).withKA(kA).withKG(kG));
					break;
			}
			return this;
		}

		public TalonFXConfig withGravityType(GravityTypeValue gravityType) {
			configuration.withSlot0(slot0Configs.withGravityType(gravityType));
			configuration.withSlot1(slot1Configs.withGravityType(gravityType));
			configuration.withSlot2(slot2Configs.withGravityType(gravityType));
			return this;
		}

		// MOTOR OUTPUT CONFIGS
		public TalonFXConfig withInvertedValue(InvertedValue invertedValue) {
			configuration.withMotorOutput(motorOutputConfigs.withInverted(invertedValue));
			return this;
		}

		public TalonFXConfig withNeutralMode(NeutralModeValue neutralMode) {
			configuration.withMotorOutput(motorOutputConfigs.withNeutralMode(neutralMode));
			return this;
		}

		// RAMP RATE CONFIGS
		public TalonFXConfig withRampRate(double rampRate) {
			closedLoopRampsConfigs
					.withDutyCycleClosedLoopRampPeriod(rampRate)
					.withTorqueClosedLoopRampPeriod(rampRate)
					.withVoltageClosedLoopRampPeriod(rampRate);
			openLoopRampsConfigs
					.withDutyCycleOpenLoopRampPeriod(rampRate)
					.withTorqueOpenLoopRampPeriod(rampRate)
					.withVoltageOpenLoopRampPeriod(rampRate);
			configuration.withClosedLoopRamps(closedLoopRampsConfigs);
			configuration.withOpenLoopRamps(openLoopRampsConfigs);
			return this;
		}

		// CURRENT LIMIT CONFIGS
		public TalonFXConfig withStatorCurrentLimitAmps(double currentLimitAmps) {
			currentLimitsConfigs
					.withStatorCurrentLimit(currentLimitAmps)
					.withStatorCurrentLimitEnable(true);
			configuration.withCurrentLimits(currentLimitsConfigs);
			return this;
		}

		public TalonFXConfig withSupplyCurrentLimitAmps(double currentLimitAmps) {
			currentLimitsConfigs
					.withSupplyCurrentLimit(currentLimitAmps)
					.withSupplyCurrentLimitEnable(true);
			configuration.withCurrentLimits(currentLimitsConfigs);
			return this;
		}

		// MOTION MAGIC CONFIGS
		public TalonFXConfig withMotionProfile(double maxVelocity, double maxAcceleration) {
			motionMagicConfigs
					.withMotionMagicCruiseVelocity(maxVelocity)
					.withMotionMagicAcceleration(maxAcceleration);
			configuration.withMotionMagic(motionMagicConfigs);
			return this;
		}

		// FEEDBACK CONFIGS
		public TalonFXConfig withRemoteSensor(
				int ID, FeedbackSensorSourceValue source, double rotorToSensorRatio) {
			feedbackConfigs
					.withFeedbackRemoteSensorID(ID)
					.withFeedbackSensorSource(source)
					.withRotorToSensorRatio(rotorToSensorRatio);
			configuration.withFeedback(feedbackConfigs);
			return this;
		}

		public TalonFXConfig withSensorToMechanismRatio(double sensorToMechanismRatio) {
			configuration.withFeedback(
					feedbackConfigs.withSensorToMechanismRatio(sensorToMechanismRatio));
			return this;
		}
	}
}
