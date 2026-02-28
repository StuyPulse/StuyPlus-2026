package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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
    private final double spokeSpacing;
    private final Color8Bit spokeColor;
    // we only have feeder, intake, and shooter visualizers. copy and paste now

    // silhouette
    private final MechanismRoot2d bumperRoot;
    private final MechanismLigament2d bumper;

    //feeder:
    private final MechanismRoot2d feederRoot;
    private final MechanismLigament2d[] feederSpokes;
    
    //intake:
    private final MechanismRoot2d intakeRoot;
    private final MechanismLigament2d intakePivot;
    private final MechanismLigament2d[] intakeTopSpokes; // for the rollers
    private final MechanismLigament2d[] intakeMiddleSpokes; // for the rollers
    private final MechanismLigament2d[] intakeBottomSpokes;

    //shooter:
    private final MechanismRoot2d shooterRoot;
    private final MechanismLigament2d[] shooterSpokes;
    
    private RobotVisualizer() {
        width = 67;
        height = 67;
        numSpokes = 5;
        spokeSpacing = 360 / numSpokes;
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

        // intake :D
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
        {
            var a = new MechanismLigament2d("_a", 8, 50, 4, new Color8Bit(Color.kGray));

            a.append(intakeBottomRollers);
            var c = new MechanismLigament2d("_c", 13.75, 95, 4, new Color8Bit(Color.kGray));
            intakeBottomRollers.append(c);

            intakeBottomRollers.append(intakeMiddleRollers);
            intakeMiddleRollers.append(intakeTopRollers);
            intakePivot.append(a);
        }

        intakeTopSpokes = createSpokes(numSpokes, intakeTopRollers, "Intake Top Spoke", .67, 2, spokeColor);
        intakeMiddleSpokes = createSpokes(numSpokes, intakeMiddleRollers, "Intake Middle Spoke", .67, 2, spokeColor);
        intakeBottomSpokes = createSpokes(numSpokes, intakeBottomRollers, "Intake Bottom Spoke", .67, 2, spokeColor);
    }

    private MechanismLigament2d[] createSpokes(int spokeNum, MechanismObject2d target, String name, double length, double width, Color8Bit color) {
        MechanismLigament2d[] spokes = new MechanismLigament2d[spokeNum];
        double spacing = 360 / numSpokes;
        for (int i = 0; i < spokeNum; i++)
            spokes[i] = target.append(new MechanismLigament2d(name.trim() + " " + i, length, spacing * i, width, color));
        return spokes;
    }

    public void updateFeeder(double RPM) {
        double rot = RPM * 6 * Settings.DT;
        for (int i = 0; i < numSpokes; i++) {
            feederSpokes[i].setAngle(feederSpokes[i].getAngle() + rot);
        }
        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateShooter(double RPM){
        double rot = RPM * 6 * Settings.DT;
        for (int i = 0; i < numSpokes; i++) {
            // ആറ് ഏഴ്
            shooterSpokes[i].setAngle(shooterSpokes[i].getAngle() + rot);
        }
        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateIntake(Rotation2d pivotAngle, double RPM) {
        intakePivot.setAngle(Rotation2d.fromDegrees(180).minus(pivotAngle)); // plus 90 degrees to make it face left
        double rot = RPM * 6 * Settings.DT;
        for (int i = 0; i < numSpokes; i++) {
            intakeTopSpokes[i].setAngle(intakeTopSpokes[i].getAngle() + rot);
            intakeMiddleSpokes[i].setAngle(intakeMiddleSpokes[i].getAngle() + rot);
            intakeBottomSpokes[i].setAngle(intakeBottomSpokes[i].getAngle() + rot);
        }

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public double getIntakeAngle() {
        return intakePivot.getAngle();
    }
}