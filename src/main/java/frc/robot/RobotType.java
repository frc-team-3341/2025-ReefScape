// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Special class that defines the type of robot used (sim, Competition, IAP,
 * etc.)
 */
public enum RobotType {

    //These offsets should be in degrees

    // 2023 IAP Robot
    ROBOT_2023_IAP_SLOTH(new double[] {
        - 62.51, //FL
        -179.82, //FR
         108.11, //BL
         82.62}),//BR

    // 2024 Competition Robot
    ROBOT_2024_COMPETITION(new double[] {
        - 12.21,   //FL
        -121.29,   //FR
        - 40.97,   //BL
        -133.154}),//BR

    //2025 Competetion Robot        
    ROBOT_2025_COMPETITION(new double[] {
        - 57.30,  //FL
           3.60,  //FR
        - 83.51,  //BL
         153.13}),//BR

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