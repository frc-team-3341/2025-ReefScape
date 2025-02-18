// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

/**
 * Use two CANSparkMaxes and CANCoder for turning and PID control 
 * using the 20 ms controller provided by WPILib.
 * 
 * @author Aric Volman
 */
public class SwerveModuleIOSparkMax {

    private SelfControlledSwerveDriveSimulation.SelfControlledModuleSimulation mapleSimModule = null;

    private SparkMax driveSparkMax;
    private SparkMax turnSparkMax;

    private SparkClosedLoopController drivePID;
    private SparkClosedLoopController turnPID;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private CANcoder canCoder;

    // Variables to store voltages of motors - REV stuff doesn't like getters
    private double driveVolts = 0.0;
    private double turnVolts = 0.0;

    // Angular offset of module - MAKE SURE CANCODER IS [-180, 180)
    private double offset = 0.0;

    // Object to hold swerve module state
    private SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0.0));

    private int num = 0;

    /**
     * @param num               Module number
     * @param driveID           CAN ID for drive motor
     * @param turnID            CAN ID for turn motor
     * @param turnCANCoderID    CAN ID for CANCoder
     * @param turnEncoderOffset Offset in degrees for module (from -180 to 180)
     * @author Aric Volman
     */
    public SwerveModuleIOSparkMax(int num, int driveID, int turnID, int turnCANCoderID, double turnEncoderOffset, boolean invert) {

        this.offset = turnEncoderOffset;
        
        this.canCoder = new CANcoder(turnCANCoderID);
        this.driveSparkMax = new SparkMax(driveID, MotorType.kBrushless);
        this.turnSparkMax = new SparkMax(turnID, MotorType.kBrushless);

        configCANCoder();
        configDriveMotor(invert);
        configTurnMotor();

        //Zero out the relative drive encoder
        this.driveEncoder.setPosition(0);
       
        // Offsets the position of the CANCoder via an offset and initializes the turning encoder
        // Offset better be in degrees
        this.turnEncoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(this.offset));
        state.angle = Rotation2d.fromRotations(getTurnPositionInRotations());

        this.num = num;

    }

    public void configureModuleSimulation(SwerveModuleSimulation simModule) {
        this.mapleSimModule = new SelfControlledSwerveDriveSimulation.SelfControlledModuleSimulation(simModule);
        this.mapleSimModule.withCurrentLimits(
            Amps.of(Constants.ModuleConstants.driveCurrentLimit),
            Amps.of(Constants.ModuleConstants.turnCurrentLimit));
    }

    private void configCANCoder() {
        //Cancoder is in rotations btw
        //CANcoderConfigurator cancoderConfigurator = canCoder.getConfigurator();
        //CANcoderConfiguration config = new CANcoderConfiguration();         
        //cancoderConfigurator.refresh(config.MagnetSensor);
    }

    private void configDriveMotor(boolean invert) {
        // Construct CANSparkMaxes
        // Initialize encoder and PID controller
        SparkMaxConfig config = new SparkMaxConfig();
        driveEncoder = driveSparkMax.getEncoder();
        drivePID = driveSparkMax.getClosedLoopController();
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        
        // Set conversion factors
        config.encoder.positionConversionFactor(ModuleConstants.drivingEncoderPositionFactor);
        config.encoder.velocityConversionFactor(ModuleConstants.drivingEncoderVelocityPositionFactor);

        // Set SPARK MAX PIDF
        // More advantageous due to 1 KHz cycle (can ramp up action quicker)
        config.closedLoop.pidf(ModuleConstants.drivekP,
                               ModuleConstants.drivekI,
                               ModuleConstants.drivekD,
                               ModuleConstants.drivekF);
        config.closedLoop.outputRange(-1, 1);
        config.inverted(invert);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ModuleConstants.driveCurrentLimit);
        config.voltageCompensation(12.0);

        //Make sure to apply the config
        driveSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configTurnMotor() {
        // Initialize encoder and PID controller
        SparkMaxConfig config = new SparkMaxConfig();
        this.turnEncoder = this.turnSparkMax.getEncoder();
        this.turnPID = this.turnSparkMax.getClosedLoopController();
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // Set conversion factors see constants for more info on what these do
        config.encoder.positionConversionFactor(ModuleConstants.turningEncoderPositionFactor);
        config.encoder.velocityConversionFactor(ModuleConstants.turningEncoderVelocityFactor);
        config.inverted(true);

        // Set SPARK MAX PIDF
        // More advantageous due to 1 KHz cycle (can ramp up action quicker)
        // This keeps the PID controller in rotations [0,1] input of 1.1 rotations will
        // end up using .1 It wraps around
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingInputRange(0, 1);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ModuleConstants.driveCurrentLimit);
        config.closedLoop.pid(ModuleConstants.turnkP,
                               ModuleConstants.turnkI,
                               ModuleConstants.turnkP);
        config.closedLoop.outputRange(-1, 1);
        
        this.turnSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getCancoderInDegrees() {
        return Units.rotationsToDegrees(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    public double getTurnPositionInRotations() {
        // Position should be already offsetted in constructor
        // Modulus is needed if the wheel is spun too much relative to the encoder's starting point
        // the min and max are taken from config.closedLoop.positionWrappingInputRange(0, 1);
        return MathUtil.inputModulus(turnEncoder.getPosition(), 0, 1);
    }

    public void setDesiredState(SwerveModuleState targetState) {
        // Optimize state so that movement is minimized
        targetState.optimize(Rotation2d.fromRotations(getTurnPositionInRotations()));
        
        // Cap setpoints at max speeds for safety
        targetState.speedMetersPerSecond = MathUtil.clamp(targetState.speedMetersPerSecond,
                -Constants.ModuleConstants.maxFreeWheelSpeedMeters, Constants.ModuleConstants.maxFreeWheelSpeedMeters);

        // Set reference of drive motor's PIDF internally in SPARK MAX
        // This automagically updates at a 1 KHz rate
        this.drivePID.setReference(targetState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);

        // Set setpoint of WPILib PID controller for turning. Will handle the turning of the motor
        // as long as we give it a target angle.
        this.turnPID.setReference(targetState.angle.getRotations(), SparkMax.ControlType.kPosition);

        // Set internal state as passed-in state
        this.state = targetState;
    }

    public SwerveModuleState getDesiredState() {
        // Returns module state
        return this.state;
    }

    public SwerveModuleState getActualModuleState() {
        return new SwerveModuleState(driveEncoder.getVelocity(),
                                     Rotation2d.fromRotations(getTurnPositionInRotations()));
    }

    public SwerveModuleState getCanCoderState(){
        return new SwerveModuleState(driveEncoder.getVelocity(),
                                     Rotation2d.fromDegrees(getCancoderInDegrees() - this.offset));
    }

    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
        this.driveVolts = volts;
    }

    public void setTurnVoltage(double volts) {
        this.turnSparkMax.setVoltage(volts);
        this.turnVolts = volts;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(),
                                        Rotation2d.fromRotations(getTurnPositionInRotations()));
    }

    public void resetEncoders() {
        // Resets only drive encoder
        driveEncoder.setPosition(0.0);
    }

    public void updateTelemetry() {
        // ESSENTIAL TELEMETRY
        // Show turning position and setpoints
        SmartDashboard.putNumber("Turn Pos Rotations#" + num, getTurnPositionInRotations());
        SmartDashboard.putNumber("Raw turn pos " + num, turnEncoder.getPosition());
        // Show driving velocity
        SmartDashboard.putNumber("Drive Vel #" + num, driveEncoder.getVelocity());

        // NON-ESSENTIAL TELEMETRY
        if (Constants.enableSwerveMotorTelemetry && num == 1) {
            /** 
             *
            // Show driving velocity setpoints
            SmartDashboard.putNumber("Setpoint Drive Vel #" + this.num, state.speedMetersPerSecond);

            // Show turning position and setpoints
            SmartDashboard.putNumber("Radian Turn Pos #" + num, getTurnPositionInRad());
            SmartDashboard.putNumber("Rad Setpoint Turn Pos #" + this.num, state.angle.getRadians());
            SmartDashboard.putNumber("Setpoint Turn Pos Deg #" + this.num, Units.radiansToDegrees(state.angle.getRadians()));

            // Get RPMs
            SmartDashboard.putNumber("Turn RPM #" + this.num, (turnEncoder.getVelocity() / 360.0) * 60.0);
            SmartDashboard.putNumber("Drive RPS #" + this.num,
                    driveEncoder.getVelocity() / Constants.ModuleConstants.drivingEncoderPositionFactor);
            SmartDashboard.putNumber("CANCoder rotation#" + this.num, canCoder.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Drive Motor Voltage #" + this.num, driveSparkMax.getAppliedOutput());

            // Output of driving
            SmartDashboard.putNumber("Turn Volts #" + this.num, this.turnVolts);
            SmartDashboard.putNumber("Drive Volts #" + this.num, this.driveVolts);

            // Get Wheel Displacement
            SmartDashboard.putNumber("Wheel Displacement #" + this.num, getPosition().distanceMeters);
            */
        }       
    }

    public int getNum() {
        return num;
    }

}