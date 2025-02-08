package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CoralManipulator extends SubsystemBase {
    private final SparkMax coralMotor = new SparkMax(21, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(0, MotorType.kBrushless);
    RelativeEncoder rel_encoder;
    SparkClosedLoopController pidPivot;
    double degrees = 180;

    CommandXboxController cXboxController;
    public CoralManipulator(CommandXboxController xboxController) {
      cXboxController = xboxController;
      rel_encoder = pivotMotor.getEncoder();
      pidPivot = pivotMotor.getClosedLoopController();
      SparkMaxConfig config = new SparkMaxConfig();
      config.closedLoop.p(.01);
      coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command pivotUp() {
      return this.runOnce(() -> pivotMotor.set(0.3));
  }
    public Command pivotDown() {
      return this.runOnce(() -> pivotMotor.set(-0.3));
  }
  public Command pivotStop() {
    return this.runOnce(() -> pivotMotor.set(0.0));
}
    public Command stopCoral() {
        return this.runOnce(() -> coralMotor.set(0.0));
    }

    public Command intakeCoral() {
        return this.runOnce(() -> coralMotor.set(0.5));
    }

    public Command releaseCoral() {
        return this.runOnce(() -> coralMotor.set(-0.5)); 
    }

    public Command spinPivot10() {
      return this.runOnce(() -> pidPivot.setReference(0.5 , SparkMax.ControlType.kPosition));
    }

    public void periodic() {
      SmartDashboard.putNumber("Position in ticks", rel_encoder.getPosition());
    }


}


