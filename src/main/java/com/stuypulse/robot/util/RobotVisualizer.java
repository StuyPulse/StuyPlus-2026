package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotVisualizer {
    public static RobotVisualizer instance;

    static {
        instance = new RobotVisualizer();
    }

    public static RobotVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;
    private final double width, height;
    private final int numSpokes;
    private final Color8Bit spokeColor;

    // silhouette
    private final MechanismRoot2d bumperRoot;
    private final MechanismLigament2d bumper;

    //feeder:
    private final MechanismRoot2d feederRoot;
    private final MechanismLigament2d[] feederSpokes;

    //intake:
    private final MechanismRoot2d intakeRoot;
    private final MechanismLigament2d intakePivot;
    private final List<MechanismLigament2d> intakeSpokes;

    //shooter:
    private final MechanismRoot2d shooterRoot;
    private final MechanismLigament2d[] shooterSpokes;

    private RobotVisualizer() {
        width = 67;
        height = 67;
        numSpokes = 5;
        spokeColor = new Color8Bit(Color.kWhite);

        canvas = new Mechanism2d(width, height);

        // Silhouette
        bumperRoot = canvas.getRoot("Bumper Root", 10, 5);
        bumper = new MechanismLigament2d("Bumper", 50, 0, 90, new Color8Bit(Color.kRed));
        bumperRoot.append(bumper);

        // Feeder
        feederRoot = canvas.getRoot("Feeder Root", 45, 15); // TODO: figure out positioning of each root
        feederSpokes = createSpokes(numSpokes, feederRoot, "Feeder Spoke", 6.7, 2, spokeColor);

        // Shooter
        shooterRoot = canvas.getRoot("Shooter Root", 55, 45);
        shooterSpokes = createSpokes(numSpokes, shooterRoot, "Shooter Spoke", 6.7, 2, spokeColor);

        // Intake :D
        intakeRoot = canvas.getRoot("Intake Root", 15, 10);

        intakePivot = new MechanismLigament2d(
            "Intake Arm",
            9,
            IntakeState.IDLE.getTargetAngle().getDegrees(),
            4,
            new Color8Bit(Color.kGray)
        );
        intakeRoot.append(intakePivot); // TODO: Find a better way to do all of this spoke/roller business

        MechanismLigament2d intakeTopRollers = new MechanismLigament2d("Intake Top Rollers", 4, -20, 4, new Color8Bit(Color.kGray));
        MechanismLigament2d intakeMiddleRollers = new MechanismLigament2d("Intake Middle Rollers", 4, 150, 4, new Color8Bit(Color.kGray));
        MechanismLigament2d intakeBottomRollers = new MechanismLigament2d("Intake Bottom Rollers", 8, 90, 4, new Color8Bit(Color.kGray));

        intakeBottomRollers.append(intakeMiddleRollers);
        intakeMiddleRollers.append(intakeTopRollers);

        {
            var _a = new MechanismLigament2d("_a", 8, 50, 4, new Color8Bit(Color.kGray));
            _a.append(intakeBottomRollers);

            var _b = new MechanismLigament2d("_b", 13.75, 95, 4, new Color8Bit(Color.kGray));
            intakeBottomRollers.append(_b);

            intakePivot.append(_a);
        }

        intakeSpokes = Arrays.stream(new MechanismLigament2d[][]{
            createSpokes(numSpokes, intakeTopRollers, "Intake Top Spoke", .67, 2, spokeColor), 
            createSpokes(numSpokes, intakeMiddleRollers, "Intake Middle Spoke", .67, 2, spokeColor),
            createSpokes(numSpokes, intakeBottomRollers, "Intake Bottom Spoke", .67, 2, spokeColor)
        })
        .flatMap(Arrays::stream)
        .collect(Collectors.toList());
    }

    private MechanismLigament2d[] createSpokes(int num, MechanismObject2d target, String name, double length, double width, Color8Bit color) {
        MechanismLigament2d[] spokes = new MechanismLigament2d[num];
        double spacing = 360 / num;
        for (int i = 0; i < num; i++)
            spokes[i] = target.append(new MechanismLigament2d(name.trim() + " " + i, length, spacing * i, width, color));
        return spokes;
    }

    public void updateFeeder(double RPM) {
        double rot = RPM * 6 * Settings.DT;
        for (MechanismLigament2d spoke : feederSpokes)
            spoke.setAngle(spoke.getAngle() + rot);

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateShooter(double RPM){
        double rot = RPM * 6 * Settings.DT;
        // ആറ് ഏഴ്
        for (MechanismLigament2d spoke : shooterSpokes)
            spoke.setAngle(spoke.getAngle() + rot);

        SmartDashboard.putData("Visualizers/Robot", canvas);
        SmartDashboard.putNumber("Shooter/rot", rot);
    }

    public void updateIntake(Rotation2d pivotAngle, double RPM) {
        intakePivot.setAngle(Rotation2d.fromDegrees(180).minus(pivotAngle)); // 180 degrees to make it face left
        double rot = RPM * 6 * Settings.DT;
        for (MechanismLigament2d spoke : intakeSpokes)
            spoke.setAngle(spoke.getAngle() + rot);

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public double getIntakeAngle() {
        return intakePivot.getAngle();
    }
}