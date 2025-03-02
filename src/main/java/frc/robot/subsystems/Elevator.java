
package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import for SparkMax
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
//import for PID controller
import com.revrobotics.spark.SparkClosedLoopController;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;




public class Elevator extends SubsystemBase {
  private SparkMax motorE = new SparkMax(25, MotorType.kBrushless);
  private SparkClosedLoopController PIDController;
  
  private double conversionFactor = (45/5.5);
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
    
    rel_encoder.setPosition(0);
    config.closedLoop.pid(
    0.0125, //p
    0.0, //i
    0.0 //d
    );
    
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
    config.smartCurrentLimit(10);
    
    // config.apply(softLimitConfig);
    config.apply(limitSwitchConfig);
      
    motorE.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
          PIDController.setReference(setpoint * -conversionFactor, SparkMax.ControlType.kPosition);
          System.out.println("Elevator L1");
          //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        }
    });
  }
    

  public Command setHeightL2(){
    return this.run(()->{
        if (this.homedStartup){
          //L2 height is 31.875 inches
          //setting the height to be 31.875 inches 
          setpoint = 16.5;
          PIDController.setReference(setpoint * -conversionFactor, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.42);
          System.out.println("Elevator L2");
          //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        }
    });
  }

  public Command setHeightL3(){
    return this.runOnce(()->{
        if (this.homedStartup){
          //L3 height is 47.625 inches
          //setting the height to be 47.625 inches 
          setpoint = 23;
          PIDController.setReference(setpoint * -conversionFactor, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.366);
          System.out.println("Elevator L3");
        }
    });
  }

  public Command setHeightL4(){
    return this.runOnce(()->{
      if (this.homedStartup){
        //L4 height is 72 inches
          //setting the height to be 72 inches 
          setpoint = 28.5;
          PIDController.setReference(setpoint * -conversionFactor, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.8);
          System.out.println("Elevator setpoint");
        //Sets the setpoint to 10 rotations. PIDController needs to be correctly configured
        //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
      }
    });
  }

  public Command moveElevator() {
    return this.run(()->{
        input = MathUtil.applyDeadband(this.rightJoyY.getAsDouble(), .1);
        //setpoint += input;
        //PIDController.setReference(this.setpoint, SparkMax.ControlType.kPosition);
        motorE.set(input * 0.5);
    });
  }

  public Command stallElevator(){
    return this.run(()->{
        PIDController.setReference(this.currentPos, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.8);
    });
  }

  public boolean isFWDLimit() {
      return fwdLimit.isPressed();
  }
  public SparkMax getMotorE(){
    return motorE;
  }
  public void resetEncoder(){
    rel_encoder.setPosition(0);
  }


  // Called when the command is initially scheduled.
  @Override
  public void periodic(){
    if (isFWDLimit()){ //Whenever the rev limit switch is pressed, resets the encoder position
      homedStartup = true;
      this.rel_encoder.setPosition(0);
    }

    SmartDashboard.putNumber("setpoint", setpoint);
    currentPos = rel_encoder.getPosition();     
    SmartDashboard.putNumber("Current position in converted rotations",currentPos * conversionFactor);
    SmartDashboard.putBoolean("Rev Limit", revLimit.isPressed());
    SmartDashboard.putBoolean("Fwd Limit", fwdLimit.isPressed());
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putNumber("Voltage", motorE.getBusVoltage() * motorE.getAppliedOutput());
    SmartDashboard.putBoolean("Homed Since Startup?", homedStartup);
    //https://www.chiefdelphi.com/t/get-voltage-from-spark-max/344136/2
  }

}