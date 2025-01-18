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
  SparkMax hangSparkMax;

  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;

  public DigitalInput limit;

  public boolean override = false;

  RelativeEncoder encoder;

  public DeepHang() {

    SparkMaxConfig config = new SparkMaxConfig();
    hangSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hangSparkMax = new SparkMax(0, MotorType.kBrushless); //deviceID = 0?

    upperLimit = hangSparkMax.getForwardLimitSwitch();
    lowerLimit = hangSparkMax.getReverseLimitSwitch();

    limit = new DigitalInput(0); //proximity sensor?

    encoder = hangSparkMax.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (upperLimit.isPressed() || limit.get()) {
      hangSparkMax.set(0);
      override = true;  
    }
    
    if (lowerLimit.isPressed() || limit.get()) {
      hangSparkMax.set(0);
      override = true;
    }

    SmartDashboard.putBoolean("upper Limit", upperLimit.isPressed());
    SmartDashboard.putBoolean("lower Limit", lowerLimit.isPressed());

    SmartDashboard.putBoolean("proximity sensor", limit.get());
    
    SmartDashboard.putNumber("encoder position", encoder.getPosition());
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public void set(double speed) {
    if (!override) {
      hangSparkMax.set(speed);
    } else {
      hangSparkMax.set(0);
    }
  }
}
