package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.IntakeSimulationSettings;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class IntakeVisualizer {
    private final LoggedMechanism2d intakeMechanism;
    private final LoggedMechanismRoot2d intakePivot;
    private final LoggedMechanismLigament2d intakeArm;

    private boolean hopperExpanded = false;

    private static final IntakeVisualizer instance;

    static {
        if (Settings.CURRENT_MODE == Settings.Mode.SIM || Settings.CURRENT_MODE == Settings.Mode.REPLAY) {
            instance = new IntakeVisualizer();
        } else {
            instance = null;
        }
    }

    public static IntakeVisualizer getInstance() {
        return instance;
    }

    private IntakeVisualizer() {
        this.intakeMechanism = new LoggedMechanism2d(1.0, 1.0, new Color8Bit(36, 36, 36));
        this.intakePivot = intakeMechanism.getRoot("Intake Pivot", 0.5, 0.3);
        this.intakeArm = intakePivot.append(
            new LoggedMechanismLigament2d(
                "Intake Arm", 
                IntakeSettings.Pivot.PIVOT_ARM_LENGTH, 
                IntakeSettings.Pivot.INITIAL_ANGLE, 
                6.0, 
                new Color8Bit(0, 180, 255)
            )
        );
    }

    public void update(Angle pivotAngle) {
        intakeArm.setAngle(pivotAngle);
        
        Pose3d intakePose = new Pose3d(
            IntakeSimulationSettings.PIVOT_OFFSET.getTranslation(),
            IntakeSimulationSettings.PIVOT_OFFSET.getRotation().plus(
                new Rotation3d(
                    0,
                    pivotAngle.in(Radians),
                    0
                )
            )
        );
        Logger.recordOutput("Intake/MechanismPose", intakePose);

        double armEndX = IntakeSimulationSettings.PIVOT_END_X
            + IntakeSettings.Pivot.PIVOT_ARM_LENGTH.in(Meters)
                * Math.sin(pivotAngle.in(Radians));

        Transform3d hopperOffset = IntakeSimulationSettings.HOPPER_OFFSET;

        double hopperX;
        if (hopperExpanded) {
            hopperX = IntakeSimulationSettings.EXPANDED_HOPPER_X;
        } else {
            hopperX = Math.max(
                IntakeSimulationSettings.MIN_HOPPER_X,
                armEndX + hopperOffset.getX());
        }

        Pose3d hopperPose = new Pose3d(
            hopperX,
            hopperOffset.getY(),
            hopperOffset.getZ(),
            hopperOffset.getRotation()
        );

        Logger.recordOutput("Hopper/MechanismPose", hopperPose);
        if (!hopperExpanded) {
            if (Math.abs(hopperX - IntakeSimulationSettings.EXPANDED_HOPPER_X) <= IntakeSimulationSettings.EXPANDED_TOLERANCE) {
                hopperExpanded = true;
            }
        }
    }
}
