
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
  private SparkMax motorE = new SparkMax(25, MotorType.kBrushless);
  private SparkClosedLoopController PIDController;
  
  private double conversionFactor = (45/5.5);
  private RelativeEncoder rel_encoder;

  private double input;
  private double currentPos;

  private double setpoint;
  
  /* Thejas Math!
  double axleD = 0.125;
  double distance = 10;
  double circ = Math.PI * axleD;
  double revolutions = distance/circ;
  double max = 50;
  double min = 0;
  */
  
  private SparkLimitSwitch revLimit;
  private SparkLimitSwitch fwdLimit;
  private DoubleSupplier rightJoyY;
  
  /** Creates a new Elevator. */
  public Elevator(DoubleSupplier rightJoyY) {
    setpoint = 0;
    
    this.rightJoyY = rightJoyY;
    double topSoftLimit = 591;
    SparkMaxConfig config = new SparkMaxConfig();
    this.PIDController = motorE.getClosedLoopController();
    this.rel_encoder = motorE.getEncoder();
    
    rel_encoder.setPosition(0);
    config.closedLoop.pidf(.0125, //p
                           0, //i
                           0, //d
                           .001);//f
    
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
    
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.forwardLimitSwitchEnabled(true);
    limitSwitchConfig.reverseLimitSwitchEnabled(true);

    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit
    softLimitConfig.forwardSoftLimit(topSoftLimit);
    softLimitConfig.reverseSoftLimit(0);

    revLimit = motorE.getReverseLimitSwitch();
    fwdLimit = motorE.getForwardLimitSwitch();
   

    // - is down and + is up
    //applies the soft limit configuration to the motor controller
    config.smartCurrentLimit(3);
    
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
    


    //   resets encoders
    public Command homing(){
        return this.run(() ->{
            while (!isREVPressed()){
                motorE.set(-0.2);
            }
            rel_encoder.setPosition(0);
        });
    }
  
    public Command setHeightL1(){
        return this.runOnce(()->{
            //motorE.set(0);
            // PIDController.setReference(106.55, SparkMax.ControlType.kPosition);
            PIDController.setReference(10 *conversionFactor, SparkMax.ControlType.kPosition);
            System.out.println("Elevator L1");
            //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        });
    }
    

    public Command setHeightL2(){
        return this.runOnce(()->{
            //motorE.set(0);
            // PIDController.setReference(261.27, SparkMax.ControlType.kPosition);
            System.out.println("Elevator L2");
            //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        });
    }

    public Command setHeightL3(){
        return this.runOnce(()->{
            motorE.set(0);
            // PIDController.setReference(295.082, SparkMax.ControlType.kPosition);
            System.out.println("Elevator L3");
        });
    }

    public Command setHeightL4(){
        return this.runOnce(()->{
        motorE.set(0);
        PIDController.setReference(10 * this.conversionFactor, SparkMax.ControlType.kPosition);
        System.out.println("Elevator setpoint");
        //Sets the setpoint to 10 rotations. PIDController needs to be correctly configured
        //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        });
    }

    public Command moveElevator() {
        return this.run(()->{
            input = this.rightJoyY.getAsDouble();
            motorE.set(input * 0.3);
        });
    }

    public boolean isREVPressed() {
        return revLimit.isPressed();
    }


    // Called when the command is initially scheduled.
  @Override
  public void periodic(){
    SmartDashboard.putNumber("setpoint", setpoint);
    currentPos = rel_encoder.getPosition();     
    SmartDashboard.putNumber("Current position in converted rotations",currentPos * conversionFactor);
    SmartDashboard.putBoolean("Rev Limit", isREVPressed());
    SmartDashboard.putBoolean("Fwd Limit", fwdLimit.isPressed());
  }

}
