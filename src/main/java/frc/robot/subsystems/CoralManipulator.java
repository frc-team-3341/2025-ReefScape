package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class CoralManipulator extends SubsystemBase {
    private final SparkMax coralMotor = new SparkMax(21, MotorType.kBrushless);
    CommandJoystick cJoystick;
    public CoralManipulator(CommandJoystick joystick) {
      cJoystick = joystick;
        SparkMaxConfig config = new SparkMaxConfig();
        coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
}
