// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Special class that defines the type of robot used (sim, Competition, IAP,
 * etc.)
 */
public enum RobotType {

    //These offsets should be in degrees will need to fix these

    // 2023 IAP Robot
    //-62.51, -179.82, 82.62, 108.11
    //-0.173639, -0.4995, 0.2295, 0.300306
    ROBOT_2023_IAP_SLOTH(new double[] {-62.51, -179.82, 82.62, 108.11}),

    // 2024 Competition Robot
    ROBOT_2024_COMPETITION(new double[] {-12.21, -121.29, -133.154, -40.97}),

    //These ones are in rotations
    //2025 Competetion Robot             FL        FR       BR        BL
    ROBOT_2025_COMPETITION(new double[] {-0.15918, 0.01001, 0.42537, -0.23196}),

    //2025 Competetion Robot                     FL      FR    BR       BL 
    ROBOT_2025_COMPETITION_DEGREES(new double[] {-57.30, 3.60, 153.13, -83.51}),

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