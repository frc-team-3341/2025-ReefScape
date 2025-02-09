// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepHang extends SubsystemBase {
  /** Creates a new DeepHang. */
  SparkMax deepHang;

  AHRS imu;

  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;

  public DigitalInput inductiveSensor;

  //2 limit switches
  //1 induction sensor -- 
  //hit one of the poles closest to hang mech
  //to help us align the robot.
  //when switch is on, metal is close to sensor

  private RelativeEncoder hangEncoder;

  public DeepHang() {

    imu = new AHRS(NavXComType.kMXP_SPI);

    deepHang = new SparkMax(22, MotorType.kBrushless); //CANID = 22
    hangEncoder = deepHang.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    //3 rotations = 1 lead screw rotation? 3:1:circumference of lead screw?
    //diameter of lead screw = .75 inches
    //config.encoder.positionConversionFactor((Math.PI * Units.inchesToMeters(.75)) / 3); //in rotations?
    //config.encoder.velocityConversionFactor((Math.PI * Units.inchesToMeters(.75)) / 3 / 60); //in rotations per second?
    //parameter ~ 0.0003
    config.encoder.positionConversionFactor(1/3); //in rotations
    config.encoder.velocityConversionFactor(1/3/60); //in rotations per second
    //parameter ~ 0.005

    config.idleMode(IdleMode.kBrake);

    inductiveSensor = new DigitalInput(0); //proximity sensor

    SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit

    softLimitConfig.forwardSoftLimit(2); //sets the forward soft limit to 2 rotations
    softLimitConfig.reverseSoftLimit(0); //sets the reverse soft limit to 0 rotations
    upperLimit = deepHang.getForwardLimitSwitch();
    lowerLimit = deepHang.getReverseLimitSwitch();

    //applies the soft limit configuration to the motor controller
    config.apply(softLimitConfig);

    //configures the motor controller with the specified configuration
    deepHang.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
  }

  public double getLinearPosition() {
    return hangEncoder.getPosition() * 0.2; //1 rotation of the encoder translates to 0.2 inches of height
  }

  public double getLinearVelocity() {
    return hangEncoder.getVelocity() * 0.2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(lowerLimit.isPressed()) {
      setSpeed(0);
      resetEncoder();
    }

    if(upperLimit.isPressed()) {
      setSpeed(0);
    }

    SmartDashboard.putNumber("Encoder Position", hangEncoder.getPosition()); //in rotations
    SmartDashboard.putNumber("Encoder Velocity", hangEncoder.getVelocity()); //in rotations per second

    SmartDashboard.putNumber("Linear Position", getLinearPosition()); //in inches
    SmartDashboard.putNumber("Linear Velocity", getLinearVelocity()); //in inches per second

    SmartDashboard.putNumber("Voltage", deepHang.getAppliedOutput()); //in volts
    SmartDashboard.putNumber("Current", (int) deepHang.getOutputCurrent()); //in amps

    // returns true if the circut is closed -- when a metalic object is close to the sensor
    SmartDashboard.putBoolean("Inductive Sensor", !inductiveSensor.get());

    SmartDashboard.putBoolean("Upper Limit", upperLimit.isPressed());
    SmartDashboard.putBoolean("Lower Limit", lowerLimit.isPressed());

    //logs the tilt of the chassis relative to the ground
    SmartDashboard.putNumber("Pitch", imu.getPitch());
  }

  public void resetEncoder() {
    hangEncoder.setPosition(0);
  }

  public void setSpeed(double setPoint) {
    deepHang.set(setPoint);
  }

  public Command fwd() {
    return this.runOnce(() -> {
      this.setSpeed(0.2); //was 0.05
    });
  }

  public Command rev() {
    return this.runOnce(() -> {
      this.setSpeed(-0.2); //was -0.05
    });
  }

  public Command stop() {
    return this.runOnce(() -> {
      this.setSpeed(0);
    });
  }
}