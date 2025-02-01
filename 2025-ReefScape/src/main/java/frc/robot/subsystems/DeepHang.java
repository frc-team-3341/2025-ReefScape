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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepHang extends SubsystemBase {
  /** Creates a new DeepHang. */
  SparkMax deepHang;

  // private double hangkP = 0.1; //initial test value
  // private double hangkI = 0.0;
  // private double hangkD = 0.0;

  // private double hangkF = 0.0;

  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;

  //public SparkClosedLoopController hangPID;

  public DigitalInput input;
  
  //2 limit switches
  //1 induction sensor -- 
  //hit one of the poles closest to hang mech
  //to help us align the robot.
  //when switch is on, metal is close to sensor

  private RelativeEncoder hangEncoder;

  public DeepHang() {

    deepHang = new SparkMax(22, MotorType.kBrushless); //CANID = 22
    hangEncoder = deepHang.getEncoder();
    //hangPID = deepHang.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Set conversion factors
    //config.encoder.positionConversionFactor(ModuleConstants.drivingEncoderPositionFactor);
    //config.encoder.velocityConversionFactor(ModuleConstants.drivingEncoderVelocityPositionFactor);
  
    //config.closedLoop.pidf(hangkP, hangkI, hangkD, hangkF);
    //config.closedLoop.outputRange(-1, 1);

    config.idleMode(IdleMode.kBrake);

    upperLimit = deepHang.getForwardLimitSwitch();
    lowerLimit = deepHang.getReverseLimitSwitch();

    input = new DigitalInput(0); //proximity sensor

    deepHang.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getEncoderPosition() {
    return hangEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return hangEncoder.getVelocity();
  }

  public double getHeight() {
    return getEncoderPosition() * 0.2; //in inches
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(lowerLimit.isPressed()) {
      hangEncoder.setPosition(0);
    }

    SmartDashboard.putNumber("Encoder Speed", (int) getEncoderVelocity());
    SmartDashboard.putNumber("Encoder Position", (int) getEncoderPosition());
    SmartDashboard.putNumber("Encoder Position", getEncoderPosition());
    SmartDashboard.putNumber("Encoder Velocity", getEncoderVelocity());

    SmartDashboard.putNumber("Height (in)", getHeight());

    SmartDashboard.putBoolean("Proximity Sensor", input.get());
    SmartDashboard.putBoolean("Upper Limit is Pressed", upperLimit.isPressed());
    SmartDashboard.putBoolean("Lower Limit is Pressed", lowerLimit.isPressed());

  }

  public void resetEncoder() {
    hangEncoder.setPosition(0);
  }

  public void setSpeed(double setPoint) {
    deepHang.set(setPoint);
  }


  public Command fwd() {
    return this.runOnce(() -> {
      this.setSpeed(0.05);
    });
  }

  public Command back() {
    return this.runOnce(() -> {
      this.setSpeed(-0.05);
    });
  }

  public Command reset() {
    return this.runOnce(() -> {
      this.setSpeed(0);
    });
  }
}
