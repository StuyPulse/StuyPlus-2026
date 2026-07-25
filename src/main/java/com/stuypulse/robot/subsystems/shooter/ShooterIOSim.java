/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.simulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim extends ShooterIOTalonFXBase {
    private static final FlywheelSim shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(3),
            Settings.Shooter.J.in(KilogramSquareMeters),
            Settings.Shooter.GEAR_RATIO),
        DCMotor.getKrakenX60(3));
    private static TalonFXSimulation getShooterMotor(int id) {
        return new TalonFXSimulation(id, shooterSim);
    }

    private final TalonFXSimulation shooterMotorLeft;
    private final TalonFXSimulation shooterMotorCenter;
    private final TalonFXSimulation shooterMotorRight;

    public ShooterIOSim() {
        this(getShooterMotor(Ports.Shooter.SHOOTER_MOTOR_RIGHT),
            getShooterMotor(Ports.Shooter.SHOOTER_MOTOR_CENTER),
            getShooterMotor(Ports.Shooter.SHOOTER_MOTOR_LEFT));
    }

    private ShooterIOSim(TalonFXSimulation shooterMotorRight, TalonFXSimulation shooterMotorCenter, TalonFXSimulation shooterMotorLeft) {
        super(shooterMotorRight, shooterMotorCenter, shooterMotorLeft);
        this.shooterMotorRight = shooterMotorRight;
        this.shooterMotorCenter = shooterMotorCenter;
        this.shooterMotorLeft = shooterMotorLeft;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterMotorRight.update(Settings.DT);
        shooterMotorCenter.update(Settings.DT);
        shooterMotorLeft.update(Settings.DT);
        super.updateInputs(inputs);
    }
}
