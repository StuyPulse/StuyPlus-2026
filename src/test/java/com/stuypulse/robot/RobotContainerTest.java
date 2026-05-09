import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import com.stuypulse.robot.RobotContainer;

public class RobotContainerTest {
    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    public void testRobotContainer() {
        assertDoesNotThrow(() -> {
            new RobotContainer();
        });
    }
}