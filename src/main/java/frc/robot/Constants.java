// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Checks if robot is real or not
    public static boolean isSim = Robot.isSimulation();
    public static boolean enableSwerveMotorTelemetry = true;
    public static boolean xboxEnabled = true;
    public static boolean invertSpeedControl = false;
    public static boolean allianceEnabled = false;

    // MODIFY THIS WHEN SWITCHING BETWEEN CHASSIS
    // THIS IS THE FIRST THING YOU SHOULD THINK ABOUT/SEE!!!

    private static final RobotType ROBOT_TYPE = RobotType.ROBOT_SIMULATION;

    public static final class SwerveModuleIOConfig{
        // Drive can ids start at front left from 1 and are odd, then go clockwise
        // Turn can ids start at front left from 2 and are even, then go clockwise
        // CANCoder ids start at front left from 10 and are sequential, then go clockwise
        // What about 9 you may ask? We need to reserve this since the new PDP usea that id
        static SwerveModuleIOSparkMax module0 = new SwerveModuleIOSparkMax(//front left
                0, 1,2,10,ROBOT_TYPE.moduleAngleOffsets[0],false);
                //num // driveID // turnID // turnCANCoderID // turnEncoderOffset // invert
        static SwerveModuleIOSparkMax module1 = new SwerveModuleIOSparkMax(//front right
                1, 3,4,11,ROBOT_TYPE.moduleAngleOffsets[1],true);
                //num // driveID // turnID // turnCANCoderID // turnEncoderOffset // invert
        static SwerveModuleIOSparkMax module2 = new SwerveModuleIOSparkMax(//back left
                2, 5,6,12,ROBOT_TYPE.moduleAngleOffsets[2],false);
                //num // driveID // turnID // turnCANCoderID // turnEncoderOffset // invert
        static SwerveModuleIOSparkMax module3 = new SwerveModuleIOSparkMax(//back right
                3, 7,8,13,ROBOT_TYPE.moduleAngleOffsets[3],true);
                //num // driveID // turnID // turnCANCoderID // turnEncoderOffset // invert
    }

    public static final class SwerveConstants {
        // These can be safely adjusted without adjusting discrete
        // Some fudge factor is needed for safety while translating + rotating
        // Max speed is 3.4 m/s
        public static final double maxChassisTranslationalSpeed = ModuleConstants.maxFreeWheelSpeedMeters; // Assuming L1 swerve
        public static final double maxWheelLinearVelocityMeters = ModuleConstants.maxFreeWheelSpeedMeters; // Assuming L1 swerve
        public static final double maxChassisAngularVelocity = Math.PI * 1.0;
        
        //distance between swerve modules on x and y axis
        public static final double swerveModuleXdistance = Units.inchesToMeters(22); 
        public static final double swerveModuleYdistance = Units.inchesToMeters(22); 
        
        //Module locations in meters
        public static final Translation2d[] moduleLocations = new Translation2d[] {
            new Translation2d( swerveModuleXdistance / 2.0,  swerveModuleYdistance / 2.0),
            new Translation2d( swerveModuleXdistance / 2.0, -swerveModuleYdistance / 2.0),
            new Translation2d(-swerveModuleXdistance / 2.0,  swerveModuleYdistance / 2.0),
            new Translation2d(-swerveModuleXdistance / 2.0, -swerveModuleYdistance / 2.0) };

        // Joystick deadband for no accidental movement
        public static final double deadBand = 0.05;

        //Probs need to update this and move it to RobotType
        public static final double robotMassInKg = 120;

        public static final double wheelGripCoefficientOfFriction = 1.19;
    }

    //We use SDS MK4i L1 modules. Google it if you want to verify all this stuff
    public static final class ModuleConstants {

        public static final int driveCurrentLimit = 35;
        public static final int turnCurrentLimit = 20;

        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0); // Assuming SDS module
        
        public static final double driveGearRatio = 8.14; // For SDS MK4i module
        public static final double turnGearRatio = 150.0/7.0; // For SDS MK4i module
        public static final double CANCoderGearRatio = 1.0; // Direct measurement

        // Both of these measurements should be correct
        // In rotations
        public static final double drivingEncoderPositionFactor = (Math.PI * wheelDiameterMeters) / driveGearRatio;
        
        // In RPM
        public static final double drivingEncoderVelocityPositionFactor = ((Math.PI * wheelDiameterMeters) / driveGearRatio) / 60.0;

        public static final double turningEncoderPositionFactor = (2 * Math.PI) / turnGearRatio; // radians
        public static final double turningEncoderVelocityFactor = (2 * Math.PI) / turnGearRatio / 60.0; // radians per second

        // Confirmed working kP!!
        public static final double drivekP = 0.1; // This is good!
        //public static final double drivekP = 0.0;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;

        // See REV: https://motors.vex.com/other-motors/neo
        // The 5790 value is the correct empirical value from the woodblocks
        // Might need to be re-calibrated for carpet or concrete
        public static final double maxRPMWoodBlocks = 5790.0;
        public static final double maxRPMCarpet = 5280.0;

        // Max free speed in RPM originally, converted to RPS native unit
        public static final double maxFreeSpeed = maxRPMCarpet / 60.0;
        // Unit for this: meters/s
        // Calculating it out:
        // 94.6 RPS * pi * 0.1016 m / 8.14 gearing = 3.7094567527 meters / s = 12.1701337 feet / s
        // Therefore, this max wheel free speed works (compared to empirical MK4i free speed)
        public static final double maxFreeWheelSpeedMeters = (maxFreeSpeed * Math.PI * wheelDiameterMeters) / driveGearRatio;
        // Unit for FF: Motor power / meters/s
        // Calculating it out: 1/3.709 = 0.26958125317 power per meters/second
        // If we want to go to the max speed of 3.709, then multiply velocity error by this constant
        // I.e. 3.709 * 0.2695 ~= 1.0

        // DO NOT MODIFY THIS UNLESS YOU KNOW WHAT YOU ARE DOING
        public static final double drivekF = 1.0/maxFreeWheelSpeedMeters;

        public static final double turnkP = 0.7; 
        public static final double turnkI = 0.0;
        public static final double turnkD = 0.0;    
    }

}
