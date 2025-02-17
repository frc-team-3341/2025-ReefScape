
package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import for SparkMax
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
//import for PID controller
import com.revrobotics.spark.SparkClosedLoopController;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;




public class Elevator extends SubsystemBase {
  final SparkMax motorE = new SparkMax(25, MotorType.kBrushless);
  SparkClosedLoopController PIDController;
  
  RelativeEncoder rel_encoder;
  CommandXboxController controller;


  double input;
  double currentPos;


  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;
 double inchesPerRevolution = 5.5;
  /** Creates a new ClimbTeleop. */
  public Elevator(CommandXboxController elevatorController) {
    controller = elevatorController;
   
    SparkMaxConfig config = new SparkMaxConfig();
    this.PIDController = motorE.getClosedLoopController();
    this.rel_encoder = motorE.getEncoder();
    config.closedLoop.p(.0085);
    double topSoftLimit = 5.5;
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    LimitSwitchConfig limitSwithConfig = new LimitSwitchConfig();
    
    limitSwithConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwithConfig.reverseLimitSwitchType(Type.kNormallyClosed);

    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit

    softLimitConfig.forwardSoftLimit(5.5);
    softLimitConfig.reverseSoftLimit(0);
    upperLimit = motorE.getForwardLimitSwitch();
    lowerLimit = motorE.getReverseLimitSwitch();

    //applies the soft limit configuration to the motor controller
    // config.smartCurrentLimit(20);
    config.apply(softLimitConfig);
    config.apply(limitSwithConfig);

    motorE.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    upperLimit = motorE.getForwardLimitSwitch();
    lowerLimit = motorE.getReverseLimitSwitch();
  }
  
 //command to stop the motor
    public Command stopElevator() {
        return this.runOnce(() -> motorE.set(0));
        
    }
  //resets encoders
  public Command homing(){
    return this.runOnce(() ->{
      while (!isREVPressed()){
        motorE.set(-0.2);
      // PIDController.setReference(0, SparkMax.ControlType.kPosition);
      }
      rel_encoder.setPosition(0);

    });
  }
  // Called when the command is initially scheduled.
  @Override
  public void periodic(){
    
   
    input = controller.getRightY();
    currentPos = rel_encoder.getPosition();     
    SmartDashboard.putNumber("Current position in ticks",currentPos);
    SmartDashboard.putNumber("CurrentPos", rel_encoder.getPosition());
    //PIDController.setReference(1, SparkMax.ControlType.kPosition);
  }


    public Command moveElevatorUp() {
      return this.runOnce(()->{
           //if (input < 0  && (!this.isFWDPressed())){
            motorE.set(input * 0.3);
         // }
        });
    }

    public Command moveElevatorDown() {
      return this.runOnce(()->{
          // if (input > 0 && (!this.isREVPressed())){
            motorE.set(input * 0.3);
          // }
        });
    }

  // public Command upPovElevator() {

  //   return this.runOnce(()->{
  //     // if (currentPos > min){
  //       motorE.set(0.3);
  //       System.out.println("Elevator up");
  //     // }     
  //   });
  // }
  // public Command downPovElevator() {
  //   return this.runOnce(()->{
  //     // if (currentPos < max) {
  //       motorE.set(-0.3);
  //       System.out.println("Elevator down");
  //     // }     
  //   });
  // }
    public Command setHeightL4(){
      return this.runOnce(()->{
        motorE.set(0);
        // PIDController.setReference(0, SparkMax.ControlType.kPosition);

        PIDController.setReference(2, SparkMax.ControlType.kPosition);
        System.out.println("Elevator setpoint");
        //Sets the setpoint to 10 rotations. PIDController needs to be correctly configured
        //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
      });
    }

  public boolean isFWDPressed(){
    return upperLimit.isPressed();
  }

  public boolean isREVPressed(){
    
    return lowerLimit.isPressed();
  }

  
  //SmartDashboard.putBoolean("Forward limit switch value", motorE.getSensorCollection().isFwdLimitSwitchClosed());
  //SmartDashboard.putBoolean("Reverse limit switch value", motorE.getSensorCollection().isRevLimitSwitchClosed());   

}
