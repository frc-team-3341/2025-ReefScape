
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

import java.util.function.DoubleSupplier;

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

  double input;
  double currentPos;

  double setpoint;
  
  double axleD = 0.125;
  double distance = 10;
  double circ = Math.PI * axleD;
  double revolutions = distance/circ;
  double max = 50;
  double min = 0;
  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;
  DoubleSupplier rightJoyY;
  
  /** Creates a new Elevator. */
  public Elevator(DoubleSupplier rightJoyY) {
    setpoint = 0;
    this.rightJoyY = rightJoyY;
    double topSoftLimit = 0.122;
    SparkMaxConfig config = new SparkMaxConfig();
    this.PIDController = motorE.getClosedLoopController();
    this.rel_encoder = motorE.getEncoder();
    config.closedLoop.pidf(.0085, //p
                           0, //i
                           0, //d
                           .001);//f
    
    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
    
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyClosed);
    limitSwitchConfig.reverseLimitSwitchType(Type.kNormallyClosed);

    softLimitConfig.forwardSoftLimitEnabled(true); //enables the forward soft limit
    softLimitConfig.reverseSoftLimitEnabled(true); //enables the reverse soft limit

    softLimitConfig.forwardSoftLimit(topSoftLimit);
    softLimitConfig.reverseSoftLimit(0);
    upperLimit = motorE.getForwardLimitSwitch();
    lowerLimit = motorE.getReverseLimitSwitch();

    config.inverted(true); // - is down and + is up
    //applies the soft limit configuration to the motor controller
    config.apply(softLimitConfig);
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
        return this.runOnce(() ->{
        while (!isREVPressed()){
            motorE.set(-0.2);
        }
        rel_encoder.setPosition(0);

        });
    }
  

 
    public Command setHeightL2(){
        return this.runOnce(()->{
            motorE.set(0);
            PIDController.setReference(261.27, SparkMax.ControlType.kPosition);
            System.out.println("Elevator L2");
            //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        });
    }

    public Command setHeightL3(){
        return this.runOnce(()->{
            motorE.set(0);
            PIDController.setReference(295.082, SparkMax.ControlType.kPosition);
            System.out.println("Elevator L3");
        });
    }

    public Command setHeightL4(){
        return this.runOnce(()->{
        motorE.set(0);
        PIDController.setReference(10, SparkMax.ControlType.kPosition);
        System.out.println("Elevator setpoint");
        //Sets the setpoint to 10 rotations. PIDController needs to be correctly configured
        //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
        });
    }

    public Command moveElevatorUp() {
        return this.run(()->{
            input = rightJoyY.getAsDouble();
            if (input < 0){
                motorE.set(input * 0.3);
            }
        });
    }

    public Command moveElevatorDown() {
        return this.run(()->{
            input = rightJoyY.getAsDouble();
            if (input > 0){
                motorE.set(input * 0.3);  
            }
        });
    }



    public boolean isREVPressed() {
        return lowerLimit.isPressed();
    }


      // Called when the command is initially scheduled.
  @Override
  public void periodic(){
    //input = rightJoyY.getAsDouble();
    SmartDashboard.putNumber("setpoint", setpoint);
    currentPos = rel_encoder.getPosition();     
    SmartDashboard.putNumber("Current position in rotations",currentPos);
  }

}
