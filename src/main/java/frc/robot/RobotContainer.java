package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.DeepHang;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  private final Joystick hangXbox = new Joystick(1);
  //private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  private final SwerveDriveTrain swerveDriveTrain = new SwerveDriveTrain(startpose,
          Constants.SwerveModuleIOConfig.module0,
          Constants.SwerveModuleIOConfig.module1,
          Constants.SwerveModuleIOConfig.module2,
          Constants.SwerveModuleIOConfig.module3);

  private final SwerveTeleopCMD swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);

  private final SwerveAutonomousCMD serveAutoCMD = new SwerveAutonomousCMD(this.swerveDriveTrain,
          Constants.allianceEnabled);
  // private TestFourModules allFour;
  // private CrabDrive crabDrive;
  
  private DeepHang hang;


  public RobotContainer() {
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
    this.configureBindings();
  }

  public void configureDeepHang() {
    hang = new DeepHang();

    POVButton hangUp = new POVButton(hangXbox, 0);
    POVButton hangDown = new POVButton(hangXbox, 180);

    hangUp.whileTrue(hang.fwd());
    hangUp.whileFalse(hang.stop());

    hangDown.whileTrue(hang.rev());
    hangDown.whileFalse(hang.stop());
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return serveAutoCMD;
    
  }

  public void initCommandInTeleop() {
    swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}