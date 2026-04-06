package com.stuypulse.robot.util.simulation;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class Simulation {
    private final static Simulation instance;

    public final Arena2026Rebuilt ARENA;

    private final SwerveDriveSimulation swerveMSim;
    private final IntakeSimulation intakeMSim;
    private final Intake intakeSim;

    private final StructArrayPublisher<Pose3d> fuel;
    private final StructPublisher<Pose3d> intakePivot;
    private final StructPublisher<Pose3d> hopper;
    private final StructPublisher<Pose3d> shooter;

    // private final Notifier simUpdateNotifier;

    static {
        instance = new Simulation();
   }

    public static Simulation getInstance() {
        return instance;
    }

    private Simulation() {
        intakeSim = Intake.getInstance();
        swerveMSim = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();

        ARENA = new Arena2026Rebuilt(false);
        configureDrivetrain(); // do this after creating arena
        intakeMSim = createIntakeSimulation();

        NetworkTableInstance table = NetworkTableInstance.getDefault();
        fuel = table.getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        intakePivot = table.getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
        hopper = table.getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();
        shooter = table.getStructTopic("AdvScope/ShooterPose", Pose3d.struct).publish();
    }

    private void configureDrivetrain() {
        ARENA.resetFieldForAuto();
        ARENA.addDriveTrainSimulation(this.swerveMSim);
        SimulatedArena.overrideInstance(ARENA);
    }

    private IntakeSimulation createIntakeSimulation() {
        return IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            swerveMSim,
            Meters.of(SimulationConstants.Intake.INTAKE_WIDTH),
            Meters.of(SimulationConstants.Intake.INTAKE_LENGTH),
            IntakeSimulation.IntakeSide.FRONT,
            SimulationConstants.Hopper.FUEL_CAPACITY
        );
    }

    private Pose3d getIntakePivotPose() {
        return SimulationConstants.Intake.PIVOT_OFFSETS.withRotation(new Rotation3d(
            0,
            intakeSim.getRelativePosition().getRadians(), // inverts the angle
            0
        ));
    }

    private double getIntakeArmEndX() {
        return SimulationConstants.Intake.PIVOT_END_X
            + SimulationConstants.Intake.PIVOT_ARM_LENGTH // sin works somehow??
            * Math.sin(intakeSim.getRelativePosition().getRadians() + SimulationConstants.Intake.PIVOT_OFFSETS.toRotation3d().getX());
    }

    private void updateIntakeEnabled(boolean enabled) {
        if (enabled) {
            intakeMSim.startIntake();
        } else {
            intakeMSim.stopIntake();
        }
    }

    private void updateIntake() {
        boolean intakeEnabled = intakeSim.atAngle() && (intakeSim.getState() == IntakeState.INTAKE);

        SmartDashboard.putBoolean("Intake/MapleSimIntakeEnabled", intakeEnabled);
        SmartDashboard.putNumber("Intake/MapleSimIntakeCurrentAmount", intakeMSim.getGamePiecesAmount());
        updateIntakeEnabled(intakeEnabled);
    }

    public synchronized void update() {
        if (swerveMSim == null) return;
        fuel.set(ARENA.getGamePiecesArrayByType("Fuel"));

        updateIntake();

        double armEndX = getIntakeArmEndX();
        intakePivot.set(getIntakePivotPose());
        hopper.set(SimulationConstants.Hopper.OFFSETS.applyToPose3d(new Pose3d(armEndX, 0, 0, new Rotation3d())));
        shooter.set(SimulationConstants.Shooter.OFFSETS.applyToPose3dRobotRelative(new Pose3d(swerveMSim.getSimulatedDriveTrainPose())));
    }
}