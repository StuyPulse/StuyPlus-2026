package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import com.stuypulse.robot.subsystems.intake.Intake.IntakeState;

public class IntakeVisualizer {
    public static IntakeVisualizer instance;

    static {
        instance = new IntakeVisualizer();
    }
    
    public static IntakeVisualizer getInstance(){
        return instance;
    }

    private final Mechanism2d canvas;
    private final MechanismLigament2d intake;
    private final MechanismRoot2d pivot;
    private double width, height;
    
    private IntakeVisualizer() {
        width = Units.inchesToMeters(20);
        height = Units.inchesToMeters(20);

        canvas = new Mechanism2d(width, height);

        intake = new MechanismLigament2d(
            "intake",
            Units.inchesToMeters(5),
            IntakeState.IDLE.getAngle(), 4,
            new Color8Bit(Color.kAliceBlue));

        pivot = canvas.getRoot("pivot", width / 2, height / 2);
        pivot.append(intake);
        
    }

    public void updatePivotAngle(Rotation2d pivotAngle) {
        SmartDashboard.putNumber("Pivot/rot", pivotAngle.getDegrees());
        intake.setAngle(pivotAngle);
        
        intake.setColor(new Color8Bit(Color.kCoral));
        SmartDashboard.putData("Pivot", canvas);
    }

    public double getAngles() {
        return intake.getAngle();
    }
}