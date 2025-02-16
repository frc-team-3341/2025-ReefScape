package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class CoralManipulator extends SubsystemBase {

//22: LEFT HAND SIDE 
//21: PIVOT POINT
//23: RIGHT HAND SIDE (looking from behind)
 

    private final SparkMax coralMotor1 = new SparkMax(22, MotorType.kBrushless);
    private final SparkMax coralMotor2 = new SparkMax(23, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(21, MotorType.kBrushless);
    AbsoluteEncoder abs_encoder;
    SparkClosedLoopController pidPivot;
    double m1Current = coralMotor1.getOutputCurrent();
    double m2Current = coralMotor2.getOutputCurrent();
    

    public CoralManipulator() {
      abs_encoder = pivotMotor.getAbsoluteEncoder();
      SparkMaxConfig config = new SparkMaxConfig();
      SparkMaxConfig config2 = new SparkMaxConfig();
      config2.inverted(true);

      this.pidPivot = pivotMotor.getClosedLoopController();
      config.closedLoop.p(.01);
      //coralMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      coralMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      coralMotor1.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    
    public Command pivotUp() {
      return this.runOnce(() -> pivotMotor.set(0.2));
  }
    public Command pivotDown() {
      return this.runOnce(() -> pivotMotor.set(-0.2));
  }
  public Command pivotStop() {
    return this.runOnce(() -> pivotMotor.set(0.0));
}
    public Command stopCoral() {
        return this.runOnce(() -> {
          coralMotor1.set(0.0);
          coralMotor2.set(0.0); 
        });

    }

    public Command intakeCoral() {
        return this.runOnce(() -> {
          
          coralMotor1.set(.2);
          coralMotor2.set(.2);
          
        });
        

    }

    public Command releaseCoral() {
        return this.runOnce(() -> {
        
        coralMotor1.set(-0.2);
        coralMotor2.set(-0.2);
         
        });

    }

    public Command spinPivot() {
      return this.runOnce(() -> pidPivot.setReference(0.5 , SparkMax.ControlType.kPosition));
    }

    public void periodic() {
      SmartDashboard.putNumber("revolution", abs_encoder.getPosition()); 
      SmartDashboard.putNumber("Setpoint Drive Velocity", pivotMotor.getBusVoltage());
      SmartDashboard.putNumber("Setpoint Drive Velocity", coralMotor1.getBusVoltage());
      SmartDashboard.putNumber("Setpoint Drive Velocity", coralMotor2.getBusVoltage());
    }
}


