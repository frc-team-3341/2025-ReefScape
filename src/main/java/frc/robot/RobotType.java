// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Special class that defines the type of robot used (sim, Competition, IAP,
 * etc.)
 */
public enum RobotType {

    // 2023 IAP Robot
    ROBOT_2023_IAP_SLOTH(new double[] {-62.51, -179.82, 108.11, 82.62}),

    // 2024 Competition Robot
    ROBOT_2024_COMPETITION(new double[] {-12.21, -121.29, -133.154, -40.97}),

    // Simulation robot
    ROBOT_SIMULATION(new double[] {0, 0, 0, 0});

    /**
     * Angular offsets of CANCoders
     */
    public double[] moduleAngleOffsets;

    private RobotType(double[] offsets) {
            this.moduleAngleOffsets = offsets;
    }

}