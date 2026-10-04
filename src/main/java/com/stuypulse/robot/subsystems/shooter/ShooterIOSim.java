/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim extends ShooterIOTalonFXBase {
    private static final SystemSim<FlywheelSim> shooterSim = SystemSim.of(new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(3),
            ShooterConstants.ShooterSettings.J.in(KilogramSquareMeters),
            ShooterConstants.ShooterSettings.GEAR_RATIO),
        DCMotor.getKrakenX60(3)));
    private static TalonFXSimulation getShooterMotor(int id) {
        return new TalonFXSimulation(id, ShooterConstants.ShooterSettings.GEAR_RATIO, shooterSim);
    }

    private final TalonFXSimulation shooterMotorLeft;
    private final TalonFXSimulation shooterMotorCenter;
    private final TalonFXSimulation shooterMotorRight;

    public ShooterIOSim() {
        this(getShooterMotor(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_RIGHT),
            getShooterMotor(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_CENTER),
            getShooterMotor(ShooterConstants.ShooterDeviceIds.SHOOTER_MOTOR_LEFT));
    }

    private ShooterIOSim(TalonFXSimulation shooterMotorRight, TalonFXSimulation shooterMotorCenter, TalonFXSimulation shooterMotorLeft) {
        super(shooterMotorRight, shooterMotorCenter, shooterMotorLeft);
        this.shooterMotorRight = shooterMotorRight;
        this.shooterMotorCenter = shooterMotorCenter;
        this.shooterMotorLeft = shooterMotorLeft;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterSim.update(Settings.DT);
        shooterMotorRight.refresh();
        shooterMotorCenter.refresh();
        shooterMotorLeft.refresh();
        super.updateInputs(inputs);
    }
}
