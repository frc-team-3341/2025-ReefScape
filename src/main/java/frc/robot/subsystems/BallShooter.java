// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.Logger;
//randome line of code so that it pushes to github
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class BallShooter extends SubsystemBase {
  private BangBangController bang = new BangBangController();
  public final WPI_TalonSRX m_flywheel = new WPI_TalonSRX(1);
  public final WPI_TalonSRX m_feedwheel = new WPI_TalonSRX(2);
  public double ticks2RPS = 4096/10;
  public double setPoint;
  private SimpleMotorFeedforward feedF = new SimpleMotorFeedforward(1,2,3);
  private boolean onOrOff;
  DigitalInput beamBreaktop = new DigitalInput(1);
  DigitalInput beamBreakbottom = new DigitalInput(2);

  /** Creates a new BallShooter. */
  public BallShooter() {
   m_flywheel.configFactoryDefault();
   m_feedwheel.configFactoryDefault();
   m_flywheel.setInverted(false);
   resetEncoders();
   m_flywheel.setNeutralMode(NeutralMode.Coast);
   m_flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public double getRPS(){
    return m_flywheel.getSelectedSensorVelocity()/ticks2RPS*-1;
  }

  public void resetEncoders(){
    m_flywheel.setSelectedSensorPosition(0, 0, 10);
  }

  public Command runMotors() {
    return this.runOnce(() -> {
      m_flywheel.set(200);
    })
  }

  public Command stopMotors() {
    return this.runOnce(() -> {
      m_flywheel.set(0); 
      m_feedwheel.set(0);
    });
  }

  public Command setFlyWheelSpeed(double setpoint) {
    if (setpoint <= 0) {
      System.out.println("Setpoint must be a non zero positive value");
      return null;
    }
    return this.runOnce(() -> {
      double voltage = bang.calculate(getRPS(), setPoint) + feedF.calculate(setPoint);
      m_flywheel.setVoltage(voltage);
    });
  }

  public Command setFeedWheelSpeed(double setpoint) {
    if (setpoint <= 0) {
      System.out.println("Setpoint must be a non zero positive value");
      return null;
    }
    return this.runOnce(() -> m_feedwheel.set(setpoint));
  }

  // public void setSpeed(double setPoint){
  //   flywheel.set(ControlMode.PercentOutput, bang.calculate(getRPS(), setPoint));
  // }

  private void setSpeedSimple(double setPoint){
    m_flywheel.set(ControlMode.PercentOutput, setPoint);
  }

  private void setFeedSimple(double setPoint){
    m_feedwheel.set(ControlMode.PercentOutput, setPoint);
  }

  private void setSpeedff(double setPoint){
    if(setPoint == 0){
      m_flywheel.set(ControlMode.PercentOutput, setPoint);
    }
    else{
      double voltage = bang.calculate(getRPS(), setPoint) + feedF.calculate(setPoint);
      m_flywheel.setVoltage(voltage);
    }
  }
    
  

  private void setOnlyFF(double setPoint){
    m_flywheel.setVoltage(feedF.calculate(setPoint)/12.0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  public void periodic() {
    // if(RobotContainer.getJoy().getRawButtonPressed(1)){
    //   onOrOff =!onOrOff;
    //   if(onOrOff){
    //     setFeedSimple(0.9);
    //   }else{
    //     setFeedSimple(0);
    //   }
    //    }
    // if(RobotContainer.getJoy().getRawButtonPressed(2)){
    //   setPoint = 0;

    // }
    // if(RobotContainer.getJoy().getRawButtonPressed(3)){
      
    //   setPoint = 97;
     
    // }
    // if(RobotContainer.getJoy().getRawButtonPressed(4)){
    //   setPoint = 90;
    // }
    // if(RobotContainer.getJoy().getRawButtonPressed(5)){
    //   setPoint = 85;
    // }
    // if(RobotContainer.getJoy().getRawButtonPressed(8)){
    //   setPoint = 50;
    // }
    // if(RobotContainer.getJoy().getRawButtonPressed(10)){
    //   setPoint = 10;
    // }
    // setSpeedff(setPoint);
    SmartDashboard.putNumber("RPS", getRPS());
    SmartDashboard.putNumber("Flyfeed Motor Voltage" , m_feedwheel.getMotorOutputVoltage());
    SmartDashboard.putNumber("Flywheel Motor Voltage" , m_flywheel.getMotorOutputVoltage());
    SmartDashboard.putNumber("setpoint", setPoint);
    SmartDashboard.putBoolean("atSetpoint", bang.atSetpoint());
    SmartDashboard.putNumber("bang", bang.calculate(getRPS(), setPoint));
    SmartDashboard.putNumber("Feed Forward", 0.9 * feedF.calculate(setPoint)/12.0);
    SmartDashboard.putNumber("flywheel (top) encoder",m_flywheel.getSelectedSensorPosition(0));
  }

  // Called once the command ends or is interrupted.
}
