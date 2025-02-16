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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  private RelativeEncoder hangEncoder;

  SendableChooser<Double> setPointChooser = new SendableChooser<>();

  public double setPoint;

  public boolean reverse;

  public DeepHang() {

    imu = new AHRS(NavXComType.kMXP_SPI);

    deepHang = new SparkMax(22, MotorType.kBrushless); //CANID = 22
    hangEncoder = deepHang.getEncoder();

    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config.idleMode(IdleMode.kBrake);

    inductiveSensor = new DigitalInput(0); //proximity sensor

    SoftLimitConfig softLimitConfig = new SoftLimitConfig();

    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit

    softLimitConfig.forwardSoftLimit(0); //sets the forward soft limit to 0 rotations
    softLimitConfig.reverseSoftLimit(20); //sets the reverse soft limit to 20 rotations

    upperLimit = deepHang.getForwardLimitSwitch();
    lowerLimit = deepHang.getReverseLimitSwitch();

    //applies the soft limit configuration to the motor controller
    config.apply(softLimitConfig);

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

    if(lowerLimit.isPressed()) {
      setSpeed(0);
    }

    if(upperLimit.isPressed()) {
      setSpeed(0);
      resetEncoder(); //tells the encoder that the current position is the zero position
    }

    // SmartDashboard.putNumber("Encoder Position", hangEncoder.getPosition()); //in rotations
    // SmartDashboard.putNumber("Encoder Velocity", hangEncoder.getVelocity()); //in rotations per second

    SmartDashboard.putNumber("Linear Position", getLinearPosition()); //in inches
    SmartDashboard.putNumber("Linear Velocity", getLinearVelocity()); //in inches per second

    // SmartDashboard.putNumber("Voltage", deepHang.getAppliedOutput()); //in volts
    // SmartDashboard.putNumber("Current", (int) deepHang.getOutputCurrent()); //in amps

    // returns true if the circut is closed -- when a metalic object is close to the sensor
    // SmartDashboard.putBoolean("Inductive Sensor", !inductiveSensor.get());

    // SmartDashboard.putBoolean("Upper Limit", upperLimit.isPressed());
    // SmartDashboard.putBoolean("Lower Limit", lowerLimit.isPressed());

    //logs the tilt of the chassis relative to the ground
    SmartDashboard.putNumber("Pitch", imu.getPitch());
    SmartDashboard.putNumber("Roll", imu.getRoll());
    SmartDashboard.putNumber("Tilt",
    Math.sqrt(Math.pow(imu.getPitch(), 2) + Math.pow(imu.getRoll(), 2)));

    setPointChooser.addOption("0.0", 0.0);
    setPointChooser.addOption("0.1", 0.1);
    setPointChooser.addOption("0.2", 0.2);
    setPointChooser.addOption("0.3", 0.3);
    setPointChooser.addOption("0.4", 0.4);
    setPointChooser.addOption("0.5", 0.5);

    SmartDashboard.putData("Set Points", setPointChooser); 
  }

  public void resetEncoder() {
    hangEncoder.setPosition(0);
  }

  public void setSpeed(double setPoint) {
    deepHang.set(setPoint);
  }

  public void setSpeed() {
    double newSetPoint = setPointChooser.getSelected();
    if (newSetPoint != setPoint){
      setPoint = newSetPoint;
    }
    if(reverse) {
      setPoint *= -1;
    }

    deepHang.set(setPoint);
  }

  public Command fwd() {
    return this.runOnce(() -> {
      reverse = false;
      this.setSpeed(); //cw away from where the motor is facing (inward)
    });
  }

  public Command rev() {
    return this.runOnce(() -> {
      reverse = true;
      this.setSpeed(); //ccw away from where the motor is facing (inward)
    });
  }

  public Command stop() {
    return this.runOnce(() -> {
      this.setSpeed(0);
    });
  }
}