// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

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

        //Not sure why these values are set here. Gonna go with the defaults for now
        //driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        //turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        //turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        this.driveEncoder.setPosition(0);
       
        // Offsets the position of the CANCoder via an offset and initializes the turning encoder, placed in proper scope of [-180, 180)
        this.turnEncoder.setPosition(this.canCoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(this.offset));
        state.angle = new Rotation2d(Units.rotationsToRadians(this.turnEncoder.getPosition()));

        this.num = num;

    }

    public void configureModuleSimulation(SwerveModuleSimulation simModule) {
        this.mapleSimModule = new SelfControlledSwerveDriveSimulation.SelfControlledModuleSimulation(simModule);
        this.mapleSimModule.withCurrentLimits(
            Amps.of(Constants.ModuleConstants.driveCurrentLimit),
            Amps.of(Constants.ModuleConstants.turnCurrentLimit));
    }

    private void configCANCoder() {
        CANcoderConfigurator cancoderConfigurator = canCoder.getConfigurator();
        CANcoderConfiguration config = new CANcoderConfiguration();         
        cancoderConfigurator.refresh(config.MagnetSensor);
        cancoderConfigurator.apply(config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                                                      .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        //canCoder.configFactoryDefault();

        // Set position of encoder to absolute mode
        //canCoder.setPositionToAbsolute();
        // Boot to absolute position
        //canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        // Important for matching up the offset of the angle
        // NEED THIS LINE - IF NOT, THEN WRONG OFFSETS WILL BE FOUND
        //canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
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

        // Set conversion factors
        config.encoder.positionConversionFactor(Units.radiansToRotations(ModuleConstants.turningEncoderPositionFactor));
        config.encoder.velocityConversionFactor(ModuleConstants.turningEncoderVelocityFactor);
        // Derived from: https://github.com/BroncBotz3481/YAGSL-Configs/tree/main/sds/mk4i/neo
        config.inverted(true);

        // Set SPARK MAX PIDF
        // More advantageous due to 1 KHz cycle (can ramp up action quicker)
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ModuleConstants.driveCurrentLimit);
        config.closedLoop.pidf(ModuleConstants.turnkP,
                               ModuleConstants.turnkI,
                               ModuleConstants.turnkP,
                               0);
        config.closedLoop.outputRange(-1, 1);
        
        this.turnSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getTurnPositionInRad() {
        // Position should be already offsetted in constructor
        // Modulus places measurement of relative encoder in absolute coordinates (-180 to 180 scope)
        // Modulus is needed if the wheel is spun too much relative to the encoder's starting point
        return MathUtil.angleModulus(Units.rotationsToRadians(turnEncoder.getPosition()));
        //return MathUtil.angleModulus(Units.degreesToRadians(this.canCoder.getAbsolutePosition() - this.offset));
    }

    public void setDesiredState(SwerveModuleState targetState) {
        // Optimize state so that movement is minimized
        // Should be fine optimizing with PID strategy of -180 to 180 scope, not 0 to 360 scope
        // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/src-html/edu/wpi/first/math/kinematics/SwerveModuleState.html#line.65
        targetState.optimize(new Rotation2d(getTurnPositionInRad()));
        
        // Cap setpoints at max speeds for safety
        targetState.speedMetersPerSecond = MathUtil.clamp(targetState.speedMetersPerSecond,
                -Constants.ModuleConstants.maxFreeWheelSpeedMeters, Constants.ModuleConstants.maxFreeWheelSpeedMeters);

        // Set reference of drive motor's PIDF internally in SPARK MAX
        // This automagically updates at a 1 KHz rate
        this.drivePID.setReference(targetState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);

        // Set setpoint of WPILib PID controller for turning
        // The built-in position wrapping should handle any discontinuity around the -180 to 180 range if needed
        this.turnPID.setReference(targetState.angle.getRotations(), SparkMax.ControlType.kPosition);

        // Set internal state as passed-in state
        this.state = targetState;
    }

    public SwerveModuleState getDesiredState() {
        // Returns module state
        return this.state;
    }

    public SwerveModuleState getActualModuleState() {
        double velocity = this.driveEncoder.getVelocity();
        double rotation = this.getTurnPositionInRad();
        return new SwerveModuleState(velocity, Rotation2d.fromRadians(rotation));
    }

    public SwerveModuleState getCanCoderState(){
        double velocity = this.driveEncoder.getVelocity();
        double rotation = Units.rotationsToRadians(this.canCoder.getAbsolutePosition().getValueAsDouble() - Units.degreesToRotations(this.offset));
        return new SwerveModuleState(velocity, Rotation2d.fromRadians(rotation));
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
        double position = driveEncoder.getPosition();
        double rotation = this.getTurnPositionInRad();
        return new SwerveModulePosition(position, new Rotation2d(rotation));
    }

    public void resetEncoders() {
        // Resets only drive encoder
        driveEncoder.setPosition(0.0);
    }

    public void updateTelemetry() {
        // ESSENTIAL TELEMETRY
        // Show turning position and setpoints
        SmartDashboard.putNumber("Turn Pos Deg #" + this.num, Units.radiansToDegrees(getTurnPositionInRad()));
        // Show driving velocity
        SmartDashboard.putNumber("Drive Vel #" + this.num, driveEncoder.getVelocity());

        // NON-ESSENTIAL TELEMETRY
        if (Constants.enableSwerveMotorTelemetry) {

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
        }       
    }

    public int getNum() {
        return num;
    }

}