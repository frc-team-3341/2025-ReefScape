// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepHang extends SubsystemBase {
  /** Creates a new DeepHang. */
  SparkMax deepHang;

  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;

  DigitalInput sensor;

  private RelativeEncoder hangEncoder;

  public DeepHang() {

    deepHang = new SparkMax(22, MotorType.kBrushless); //CANID = 22
    SparkMaxConfig config = new SparkMaxConfig();
    deepHang.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    upperLimit = deepHang.getForwardLimitSwitch();
    lowerLimit = deepHang.getReverseLimitSwitch();

    sensor = new DigitalInput(0); //proximity sensor

    hangEncoder = deepHang.getEncoder();

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

  public void setSpeed(double speed) {
    deepHang.set(speed);
  }
}
