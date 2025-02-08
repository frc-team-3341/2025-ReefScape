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

    ///config.encoder.positionConversionFactor(); //num * rotations
    //config.encoder.velocityConversionFactor(); //num * rotations per second

    config.idleMode(IdleMode.kBrake);

    inductiveSensor = new DigitalInput(0); //proximity sensor

    SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit

    softLimitConfig.forwardSoftLimit(60); //sets the forward soft limit to 12 inches
    softLimitConfig.reverseSoftLimit(-60); //sets the reverse soft limit to -12 inches

    upperLimit = deepHang.getForwardLimitSwitch();
    lowerLimit = deepHang.getReverseLimitSwitch();

    //applies the soft limit configuration to the motor controller
    config.apply(softLimitConfig);

    //configures the motor controller with the specified configuration
    deepHang.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
  }

  public double getEncoderPosition() {
    return hangEncoder.getPosition(); //in rotations
  }

  public double getEncoderVelocity() {
    return hangEncoder.getVelocity() / 60; //in rotations per second
  }

  public double getLinearPosition() {
    return getEncoderPosition() * 0.2; //in inches
  }

  public double getLinearVelocity() {
    return getEncoderVelocity() * 0.2; //in inches per second
  }

  public double getVoltage() {
    return deepHang.getAppliedOutput(); //in volts
  }

  public double getCurrent() {
    return deepHang.getOutputCurrent(); //in amps
  }

  public double getPitch() {
    return imu.getPitch(); //in degrees
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(lowerLimit.isPressed()) {
      setSpeed(0);
      hangEncoder.setPosition(0);
    }

    if(upperLimit.isPressed()) {
      setSpeed(0);
    }

    SmartDashboard.putNumber("Encoder Position", getEncoderPosition());
    SmartDashboard.putNumber("Encoder Velocity", getEncoderVelocity());

    SmartDashboard.putNumber("Linear Position", getLinearPosition());
    SmartDashboard.putNumber("Linear Velocity", getLinearVelocity());

    SmartDashboard.putNumber("Voltage", getVoltage());
    SmartDashboard.putNumber("Current", getCurrent());

    SmartDashboard.putBoolean("Inductive Sensor", inductiveSensor.get());

    SmartDashboard.putBoolean("Upper Limit", upperLimit.isPressed());
    SmartDashboard.putBoolean("Lower Limit", lowerLimit.isPressed());

    SmartDashboard.putNumber("Pitch", getPitch()); //logs the tilt of the chassis relative to the ground
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

  public Command back() {
    return this.runOnce(() -> {
      this.setSpeed(-0.2); //was -0.05
    });
  }

  public Command reset() {
    return this.runOnce(() -> {
      this.setSpeed(0);
    });
  }
}
