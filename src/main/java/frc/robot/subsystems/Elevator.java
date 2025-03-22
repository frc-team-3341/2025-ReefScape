package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
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
  private boolean enableTeleop;
  
  /** Creates a new Elevator. */
  public Elevator(DoubleSupplier rightJoyY) {
    
    setpoint = 0;
    this.rightJoyY = rightJoyY;
    double topSoftLimit = 50 * conversionFactor;
    SparkMaxConfig config = new SparkMaxConfig();
    this.PIDController = motorE.getClosedLoopController();
    this.rel_encoder = motorE.getEncoder();
    enableTeleop = false;
    
    config.closedLoop.pid(
    .6, //p
    0.006, //i
    0.005 //d
    ); 

    config.closedLoop.maxMotion
       .maxVelocity(5000) //in rpm
       .maxAcceleration(4400) // in rpm/s
       .allowedClosedLoopError(1);

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

  public Command toggleTeleop() {
    return this.runOnce(() -> {
        enableTeleop = !enableTeleop;
    });
}
  

  //command to stop the motor
  public Command stopElevator() {
    return this.runOnce(() -> {
        //System.out.println("stop the elevator");
        motorE.set(0);
        setpoint = 0;
    });    
  }

  public Command setHeightL1(){
    PIDController.toString();
    return this.runOnce(()->{
        if (this.homedStartup){
          //L1 height is inches
          //setting the height to be 10 inches 
          setpoint = 10;
          PIDController.setReference(setpoint * conversionFactor, SparkMax.ControlType.kMAXMotionPositionControl);
          //System.out.println("Elevator L1");
          //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        }
    });
  }

  public Command setHeightL2(){
    return this.runOnce(()->{
        if (this.homedStartup){ //2.5, 0.009
          setpoint = 3.9;
          PIDController.setReference(setpoint * conversionFactor, SparkMax.ControlType.kMAXMotionPositionControl);//, ClosedLoopSlot.kSlot0, 0.2);
          //System.out.println("Elevator L2");
          //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        }
    });
  }

  public Command setHeightL3(){
    return this.runOnce(()->{
        if (this.homedStartup){
          setpoint = 12.6;
          PIDController.setReference(setpoint * conversionFactor, SparkMax.ControlType.kMAXMotionPositionControl);//, ClosedLoopSlot.kSlot0, 0.009);
          //System.out.println("Elevator L3");
        }
    });
  }

  public Command setHeightL4(){ //41˚
    return this.runOnce(()->{
      if (this.homedStartup){ 
          setpoint = 26.5;
          PIDController.setReference(setpoint * conversionFactor, SparkMax.ControlType.kMAXMotionPositionControl);//, ClosedLoopSlot.kSlot0, 0.271);
          //System.out.println("Elevator setpoint L4");
        //Sets the setpoint to 10 rotations. PIDController needs to be correctly configured
        //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
      }
    });
  }

  public Command moveElevator() {
    return this.run(()->{
        if (enableTeleop) {
          //This joystick up is negative and down is positive so we need to invert it.
          input = MathUtil.applyDeadband(this.rightJoyY.getAsDouble(), .1);
          
          setpoint += (input * -1) * .025;
          MathUtil.clamp(setpoint, 0, 27);
          PIDController.setReference(this.setpoint * conversionFactor, SparkMax.ControlType.kPosition);
          //System.out.println(input);
          //motorE.set(input * 0.3);
        }    
    });
  }

  //Move the elevator down at a constant speed for homing
  public Command homeElevatorDown() {
    return this.runOnce(() -> {
      if (homedStartup) {
        setpoint = 0;
        PIDController.setReference(setpoint, SparkMax.ControlType.kMAXMotionPositionControl);
      } else {
        motorE.set(-0.3);
      }
    });
  }

  public boolean isREVLimit() {
    return revLimit.isPressed();
  }

  public Command resetEncoder() {
    return this.runOnce(() -> {
      //System.out.println("Elevator reset encoder");
      homedStartup = true;
      rel_encoder.setPosition(0);
    });   
  }
  
  public SparkMax getMotor() {
    return motorE;
  }

  @Override
  public void periodic(){
    // SmartDashboard.putNumber("Elevator setpoint", setpoint);
    currentPos = rel_encoder.getPosition();     
    SmartDashboard.putNumber("Elevator pos",currentPos / conversionFactor);
    // SmartDashboard.putNumber("Elevator vel", rel_encoder.getVelocity());
    SmartDashboard.putBoolean("Elevator Rev Limit", revLimit.isPressed());
    SmartDashboard.putBoolean("Elevator Fwd Limit", fwdLimit.isPressed());
    // SmartDashboard.putNumber("Elevator Voltage", motorE.getBusVoltage() * motorE.getAppliedOutput());
    SmartDashboard.putBoolean("Elevator Homed?", homedStartup);
    // SmartDashboard.putNumber("Eleavtor IAccum", PIDController.getIAccum());
    
    
    //https://www.chiefdelphi.com/t/get-voltage-from-spark-max/344136/2
  }

}