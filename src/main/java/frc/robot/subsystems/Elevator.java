package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class Elevator extends SubsystemBase {
  private SparkMax motorE = new SparkMax(25, MotorType.kBrushless);
  private SparkClosedLoopController PIDController;
  
  private double conversionFactor = (Constants.ElevatorConstants.elevatorGearRatio / 5.5);
  private RelativeEncoder rel_encoder;

  private double input;
  private double currentPos;
  private double setpoint;
  
  private SparkLimitSwitch revLimit;
  private SparkLimitSwitch fwdLimit;
  private DoubleSupplier rightJoyY;
  private boolean homedStartup = false;
  
  /** Creates a new Elevator. */
  public Elevator(DoubleSupplier rightJoyY) {
    
    setpoint = 0;
    this.rightJoyY = rightJoyY;
    double topSoftLimit = 50 * conversionFactor;
    SparkMaxConfig config = new SparkMaxConfig();
    this.PIDController = motorE.getClosedLoopController();
    this.rel_encoder = motorE.getEncoder();
    
    config.closedLoop.pid(
    10, //p
    0.0, //i
    0.0 //d
    );

    config.closedLoop.maxMotion
       .maxVelocity(5000) //in rpm
       .maxAcceleration(3000); // in rpm/s
    //   //.allowedClosedLoopError(allowedErr);
    
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
    
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.reverseLimitSwitchEnabled(true);

    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit
    softLimitConfig.forwardSoftLimit(0);
    softLimitConfig.reverseSoftLimit(topSoftLimit);

    revLimit = motorE.getReverseLimitSwitch();
    fwdLimit = motorE.getForwardLimitSwitch();

    // - is down and + is up
    //applies the soft limit configuration to the motor controller
    config.smartCurrentLimit(15);
    config.inverted(true);
    
    // config.apply(softLimitConfig);
    config.apply(limitSwitchConfig);
      
    motorE.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //We started homed make sure to reset encoder
    if (isREVLimit()) {
      this.rel_encoder.setPosition(0);
      homedStartup = true;
    }
  }
  

  //command to stop the motor
  public Command stopElevator() {
    return this.runOnce(() -> {
        motorE.set(0);
    });    
  }
  
  public Command setHeightL1(){
    return this.runOnce(()->{
        if (this.homedStartup){
          //L1 height is inches
          //setting the height to be 10 inches 
          setpoint = 10;
          PIDController.setReference(setpoint * conversionFactor, SparkMax.ControlType.kMAXMotionPositionControl);
          System.out.println("Elevator L1");
          //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        }
    });
  }
    

  public Command setHeightL2(){
    return this.runOnce(()->{
        if (this.homedStartup){ //2.5, 0.009
          setpoint = 3.9;
          PIDController.setReference(setpoint * conversionFactor, SparkMax.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.2);
          System.out.println("Elevator L2 " + setpoint*conversionFactor);
          //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        }
    });
  }

  public Command setHeightL3(){
    return this.runOnce(()->{
        if (this.homedStartup){
          setpoint = 12.6;
          PIDController.setReference(setpoint * conversionFactor, SparkMax.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.009);
          System.out.println("Elevator L3 " + setpoint*conversionFactor);
        }
    });
  }

  public Command setHeightL4(){ //41˚
    return this.runOnce(()->{
      if (this.homedStartup){ 
          setpoint = 26;
          PIDController.setReference(setpoint * conversionFactor, SparkMax.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.271);
          System.out.println("Elevator setpoint " + setpoint*conversionFactor);
        //Sets the setpoint to 10 rotations. PIDController needs to be correctly configured
        //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
      }
    });
  }

  public Command moveElevator() {
    return this.run(()->{
        input = MathUtil.applyDeadband(this.rightJoyY.getAsDouble(), .1);
        setpoint += input;
        PIDController.setReference(this.setpoint, SparkMax.ControlType.kMAXMotionPositionControl);
        //motorE.set(input * 0.5);
    });
  }

  //Move the elevator down at a constant speed for homing
  public Command homeElevatorDown() {
    return this.runOnce(() -> {
      if (homedStartup) {
        PIDController.setReference(0, SparkMax.ControlType.kMAXMotionPositionControl);
      } else {
        motorE.set(-0.3);
      }
    });
  }

  public Command stallElevator(){
    return this.run(()->{
        PIDController.setReference(this.currentPos, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.8);
    });
  }

  public boolean isREVLimit() {
    return revLimit.isPressed();
  }

  public Command resetEncoder() {
    return this.runOnce(() -> {
      System.out.println("RESET");
      homedStartup = true;
      rel_encoder.setPosition(0);
    });   
  }
  
  public SparkMax getMotor() {
    return motorE;
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("setpoint", setpoint);
    currentPos = rel_encoder.getPosition();     
    SmartDashboard.putNumber("Current position in converted rotations",currentPos / conversionFactor);
    SmartDashboard.putBoolean("Rev Limit", revLimit.isPressed());
    SmartDashboard.putBoolean("Fwd Limit", fwdLimit.isPressed());
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putNumber("Voltage", motorE.getBusVoltage() * motorE.getAppliedOutput());
    SmartDashboard.putBoolean("Homed Since Startup?", homedStartup);
    SmartDashboard.putNumber("Left Slider", rightJoyY.getAsDouble());
    
    //https://www.chiefdelphi.com/t/get-voltage-from-spark-max/344136/2
  }

}