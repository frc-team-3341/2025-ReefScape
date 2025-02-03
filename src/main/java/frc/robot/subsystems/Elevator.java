
package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import for SparkMax
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
//import for PID controller
import com.revrobotics.spark.SparkClosedLoopController;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;




public class Elevator extends SubsystemBase {
  final SparkMax motorE = new SparkMax(20, MotorType.kBrushless);
  SparkClosedLoopController PIDController;
  
  RelativeEncoder rel_encoder;
  CommandXboxController controller;


  double input;
  double currentPos;
  
  double axleD = 0.125;
  double distance = 10;
  double circ = Math.PI * axleD;
  double revolutions = distance/circ;
  double max = 100;
  double min = -100;
  
  /** Creates a new ClimbTeleop. */
  public Elevator() {
    SparkMaxConfig config = new SparkMaxConfig();
    this.PIDController = motorE.getClosedLoopController();
    this.rel_encoder = motorE.getEncoder();
    config.closedLoop.p(.01);
    motorE.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  
 //command to stop the motor
    public Command stopElevator() {
        return this.runOnce(() -> motorE.set(0));
        
    }
    
  //resets encoders
  public void initialize(){
      currentPos = 0;
    }
  
  
  
  
  // Called when the command is initially scheduled.
  @Override
  public void periodic(){
    
   
    input = controller.getRightY();
    currentPos = rel_encoder.getPosition();     
    SmartDashboard.putNumber("Current position in ticks",currentPos);
    //PIDController.setReference(1, SparkMax.ControlType.kPosition);
  }

  public Command upElevator() {

    return this.runOnce(()->{
      if ((currentPos > min) && (input > 0.2)) {
        motorE.set(input *0.3);
        System.out.println("down");
      }     
    });
  }
    public Command downElevator() {

      return this.runOnce(()->{
        if ((currentPos < max) && (input <= -0.2)) {
          motorE.set(input *0.3);
          System.out.println("down");
        }     

      });
  }
  public Command upPovElevator() {

    return this.runOnce(()->{
      // if (currentPos > min){
        motorE.set(0.3);
      // }     
    });
  }
  public Command downPovElevator() {
    return this.runOnce(()->{
      // if (currentPos < max) {
        motorE.set(-0.3);
      // }     
    });
  }
    public Command setHeightL4(){
      return this.runOnce(()->{
        motorE.set(0);
        PIDController.setReference(10, SparkMax.ControlType.kPosition);
        //Sets the setpoint to 10 rotations. PIDController needs to be correctly configured
        //https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode
      });
    }

  
  
  //SmartDashboard.putBoolean("Forward limit switch value", motorE.getSensorCollection().isFwdLimitSwitchClosed());
  //SmartDashboard.putBoolean("Reverse limit switch value", motorE.getSensorCollection().isRevLimitSwitchClosed());   
    }
