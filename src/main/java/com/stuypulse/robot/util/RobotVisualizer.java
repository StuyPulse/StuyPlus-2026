package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
    // we only have feeder, intake, and shooter visualizers. copy and paste now

    
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

        canvas = new Mechanism2d(width, height);

        // Feeder
        feederRoot = canvas.getRoot("Feeder Root", 45, 15); // TODO: figure out positioning of each root
        feederSpokes = new MechanismLigament2d[numSpokes];

        // Shooter
        shooterRoot = canvas.getRoot("Shooter Root", 55, 45);
        shooterSpokes = new MechanismLigament2d[numSpokes];

        // intake :D
        intakeRoot = canvas.getRoot("Intake Root", 15, 10);
        intakeTopSpokes = new MechanismLigament2d[numSpokes];
        intakeMiddleSpokes = new MechanismLigament2d[numSpokes];
        intakeBottomSpokes = new MechanismLigament2d[numSpokes];

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
        
        // fill all the spoke arrays
        for (int i = 0; i < numSpokes; i++) {
            feederSpokes[i] = new MechanismLigament2d("Feeder Spoke " + i, 6.7, spokeSpacing * i, 2, new Color8Bit(Color.kWhite));
            feederRoot.append(feederSpokes[i]);
            shooterSpokes[i] = new MechanismLigament2d("Shooter Spoke " + i, 6.7, spokeSpacing * i, 2, new Color8Bit(Color.kYellow));
            shooterRoot.append(shooterSpokes[i]);

            intakeTopSpokes[i] = new MechanismLigament2d("Intake Top Spoke " + i, .67, spokeSpacing * i, 2, new Color8Bit(Color.kWhite));
            intakeTopRollers.append(intakeTopSpokes[i]); // TODO: spoke lengths and thicknesses
            intakeMiddleSpokes[i] = new MechanismLigament2d("Intake Middle Spoke " + i, .67, spokeSpacing * i, 2, new Color8Bit(Color.kWhite));
            intakeMiddleRollers.append(intakeMiddleSpokes[i]);
            intakeBottomSpokes[i] = new MechanismLigament2d("Intake Bottom Spoke " + i, .67, spokeSpacing * i, 2, new Color8Bit(Color.kWhite));
            intakeBottomRollers.append(intakeBottomSpokes[i]);
        }
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