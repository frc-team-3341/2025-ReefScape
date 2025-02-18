package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CoralManipulator extends SubsystemBase {
  double setpoint;

  //22: LEFT HAND SIDE 
  //21: PIVOT POINT
  //23: RIGHT HAND SIDE (looking from behind)
 
  //Figure out which ones of these is top and bottom
  public SparkLimitSwitch upperLimit;
  public SparkLimitSwitch lowerLimit;

  private final SparkMax coralMotor1 = new SparkMax(22, MotorType.kBrushless);
  private final SparkMax coralMotor2 = new SparkMax(23, MotorType.kBrushless);

  private final SparkMax pivotMotor = new SparkMax(21, MotorType.kBrushless);
  AbsoluteEncoder absEncoderPivot;
  SparkClosedLoopController pidPivot;

  CommandXboxController controller;

  public CoralManipulator(CommandXboxController controller) {
    SparkMaxConfig config = new SparkMaxConfig();

    this.controller = controller;
    this.pidPivot = pivotMotor.getClosedLoopController();
    this.absEncoderPivot = pivotMotor.getAbsoluteEncoder();

    //Config coral motor1
    SparkMaxConfig coralConfig1 = new SparkMaxConfig();
    coralMotor1.configure(coralConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Config coral motor2
    SparkMaxConfig coralConfig2 = new SparkMaxConfig();
    coralConfig2.inverted(true);
    coralMotor2.configure(coralConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Config pivot Motor
    pidPivot = pivotMotor.getClosedLoopController();
    absEncoderPivot = pivotMotor.getAbsoluteEncoder();
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    
    pivotConfig.closedLoop.p(.01);
    pivotConfig.softLimit
        .forwardSoftLimit(50)
        .reverseSoftLimit(-50)

        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);
    upperLimit = pivotMotor.getForwardLimitSwitch();
    lowerLimit = pivotMotor.getReverseLimitSwitch();
        config.apply(pivotConfig);
        config.apply(pivotConfig);
    

    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }
    
   public Command pivotUp() {
    return this.runOnce(() -> {
      this.setpoint = 0;
      this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
    });
  }

  public Command pivotDown() {
    return this.runOnce(() -> {
      this.setpoint = 0;
      this.pidPivot.setReference(setpoint, SparkMax.ControlType.kPosition);
    });  }

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

    double val = .1 * controller.getLeftX();
    pivotMotor.set(val);


    SmartDashboard.putNumber("revolution", absEncoderPivot.getPosition()); 
    SmartDashboard.putNumber("Setpoint Drive Velocity", pivotMotor.getBusVoltage());
    SmartDashboard.putNumber("Setpoint Drive Velocity", coralMotor1.getBusVoltage());
    SmartDashboard.putNumber("Setpoint Drive Velocity", coralMotor2.getBusVoltage());
  }
  public boolean isFWDPressed() {
    return upperLimit.isPressed();
  }

  boolean isREVPressed(){
    return lowerLimit.isPressed();
}
  }
