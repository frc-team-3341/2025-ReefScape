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
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepHang extends SubsystemBase {
  /** Creates a new DeepHang. */
  private SparkMax deepHang;

  private RelativeEncoder hangEncoder;

  private SparkLimitSwitch upperLimit;
  private SparkLimitSwitch lowerLimit;

  public DeepHang() {
    deepHang = new SparkMax(30, MotorType.kBrushless);
    hangEncoder = deepHang.getEncoder();
    hangEncoder.setPosition(0);

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);

    this.upperLimit = deepHang.getForwardLimitSwitch();
    this.lowerLimit = deepHang.getReverseLimitSwitch();

    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.reverseLimitSwitchEnabled(true);
    config.apply(limitSwitchConfig);

    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit
    softLimitConfig.forwardSoftLimit(95);
    softLimitConfig.reverseSoftLimit(-95);
    //config.apply(softLimitConfig);
   
    //configures the motor controller with the specified configuration
    deepHang.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
  }

  public double getPosition(){
    return hangEncoder.getPosition() * 1/3;
  }

  public double getVelocity(){
    return hangEncoder.getPosition() * 1/3 / 60;
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
    SmartDashboard.putNumber("Encoder Position", hangEncoder.getPosition()); //in rotations
    SmartDashboard.putNumber("Encoder Velocity", hangEncoder.getVelocity()); //in rotations per second

    SmartDashboard.putNumber("Linear Position", getLinearPosition()); //in inches
    SmartDashboard.putNumber("Linear Velocity", getLinearVelocity()); //in inches per second

    SmartDashboard.putNumber("Voltage", deepHang.getAppliedOutput()); //in volts
    SmartDashboard.putNumber("Current", (int) deepHang.getOutputCurrent()); //in amps

    boolean upperBool = upperLimit.isPressed();
    boolean lowerBool = lowerLimit.isPressed();
    SmartDashboard.putBoolean("upper is pressed", upperBool);
    SmartDashboard.putBoolean("lower is pressed", lowerBool);

    // returns true if the circut is closed -- when a metalic object is close to the sensor
    // SmartDashboard.putBoolean("Inductive Sensor", !inductiveSensor.get());

    // SmartDashboard.putBoolean("Upper Limit", upperLimit.isPressed());
    // SmartDashboard.putBoolean("Lower Limit", lowerLimit.isPressed());

    //logs the tilt of the chassis relative to the ground
    //SmartDashboard.putNumber("Pitch", imu.getPitch());
    //SmartDashboard.putNumber("Roll", imu.getRoll());
    //SmartDashboard.putNumber("Tilt",
    //Math.sqrt(Math.pow(imu.getPitch(), 2) + Math.pow(imu.getRoll(), 2)));

    SmartDashboard.putNumber("Encoder Position", hangEncoder.getPosition()); //in rotations
  }

  public void resetEncoder() {
    hangEncoder.setPosition(0);
  }

  public Command fwd() {
    return this.runOnce(() -> {
      deepHang.set(.5);
    });
  }

  public Command rev() {
    return this.runOnce(() -> {
      deepHang.set(-.5);
    });
  }

  public Command stop() {
    return this.runOnce(() -> {
      deepHang.set(0);
    });
  }
}