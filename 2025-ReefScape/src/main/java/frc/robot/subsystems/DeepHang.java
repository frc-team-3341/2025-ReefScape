// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepHang extends SubsystemBase {
  /** Creates a new DeepHang. */
  SparkMax deepHang;

  private double drivekP = 0.0;
  private double drivekI = 0.0;
  private double drivekD = 0.0;

  private double drivekF = 0.0;

  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;

  public SparkClosedLoopController drivePID;

  public DigitalInput sensor;

  private RelativeEncoder hangEncoder;

  public DeepHang() {

    deepHang = new SparkMax(22, MotorType.kBrushless); //CANID = 22
    drivePID = deepHang.getClosedLoopController();
    hangEncoder = deepHang.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.pidf(drivekP, drivekI, drivekD, drivekF);
    config.closedLoop.outputRange(-1, 1);

    config.idleMode(IdleMode.kBrake);

    deepHang.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    upperLimit = deepHang.getForwardLimitSwitch();
    lowerLimit = deepHang.getReverseLimitSwitch();

    sensor = new DigitalInput(0); //proximity sensor

  }

  public double getEncoderPosition() {
    return hangEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return hangEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // if (upperLimit.isPressed()) {
    //   deepHang.set(0);  
    // }
    
    // if (lowerLimit.isPressed()) {
    //   deepHang.set(0);
    // }

    // SmartDashboard.putBoolean("upper Limit", upperLimit.isPressed());
    // SmartDashboard.putBoolean("lower Limit", lowerLimit.isPressed());
    // SmartDashboard.putBoolean("proximity sensor", sensor.get());

    SmartDashboard.putNumber("encoder speed", (int) getEncoderVelocity());
    SmartDashboard.putNumber("encoder position", (int) getEncoderPosition());
  }

  public void resetEncoder() {
    hangEncoder.setPosition(0);
  }

  public void setSpeed(double setPoint) {
    deepHang.set(setPoint);
    //drivePID.setReference(setPoint, ControlType.kVelocity);
  }
}
