/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotContainerTest {
    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    public void testRobotContainer() {
        assertDoesNotThrow(
                () -> {
                    new RobotContainer();
                });
    }
}
