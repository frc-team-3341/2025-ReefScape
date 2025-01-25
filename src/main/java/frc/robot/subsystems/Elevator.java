
package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import for SparkMax
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
//import for PID controller
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator;
import frc.robot.RobotContainer;
import frc.robot.Constants;




public class Elevator extends SubsystemBase {
  final SparkMax motorE = new SparkMax(20, MotorType.kBrushless);
  SparkMaxConfig config = new SparkMaxConfig();
  SparkAbsoluteEncoder abs_Encoder;
  Joystick joystick;
  double maxHeight = 52;
    double RPI = 20;
    double max = 100000;
    double min = -100000;
  /** Creates a new ClimbTeleop. */
  public Elevator(Joystick joystick) {
    this.joystick = joystick;
    this.abs_Encoder = motorE.getAbsoluteEncoder();
    double currentPos = abs_Encoder.getPosition();
    motorE.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
 //command to stop the motor
    public Command stopElevator() {
        return this.runOnce(() -> motorE.set(0));
    }

  //resets encoders
  public void initialize(){
      //abs_Encoder.
    }
  
  
  
  
  // Called when the command is initially scheduled.
  @Override
  public void periodic(){
    
   
    double input = joystick.getY();
    double currentPos = abs_Encoder.getPosition();

    if(currentPos < max && input <=  -.2 ||
    currentPos > min && input > .2){
      motorE.set(input * .3);
    }
    // if(joystick.getRawButtonPressed(0)) {
    //     motorE.set(0);
    

   
  }
  //SmartDashboard.putNumber("Current position in ticks",currentPos);
  //SmartDashboard.putBoolean("Forward limit switch value", motorE.getSensorCollection().isFwdLimitSwitchClosed());
  //SmartDashboard.putBoolean("Reverse limit switch value", motorE.getSensorCollection().isRevLimitSwitchClosed());   
    }
