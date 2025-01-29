package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.commands.swerve.targeting.Alignment;
import frc.robot.commands.swerve.targeting.LongitudinalAlignment;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.swerve.targeting.Vision;

import org.photonvision.PhotonCamera;

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
  //private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  private final SwerveDriveTrain swerveDriveTrain = new SwerveDriveTrain(startpose,
          Constants.SwerveModuleIOConfig.module0,
          Constants.SwerveModuleIOConfig.module1,
          Constants.SwerveModuleIOConfig.module2,
          Constants.SwerveModuleIOConfig.module3);

  private final SwerveTeleopCMD swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);

  private final SwerveAutonomousCMD serveAutoCMD = new SwerveAutonomousCMD(this.swerveDriveTrain,
          Constants.allianceEnabled);

  private final Alignment align;
  private final LongitudinalAlignment longAlignment;
  private final Vision vision;

  private final PhotonCamera cam = new PhotonCamera("camera");
  // private TestFourModules allFour;
  // private CrabDrive crabDrive;



  public RobotContainer() {
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
    vision = new Vision(cam);
    longAlignment = new LongitudinalAlignment(swerveDriveTrain, vision);
    align = new Alignment(swerveDriveTrain, vision);
    JoystickButton alignButton = new JoystickButton(drivingXbox, XboxController.Button.kA.value);
    JoystickButton longAlignButton = new JoystickButton(drivingXbox, XboxController.Button.kX.value);

    alignButton.toggleOnTrue(new SequentialCommandGroup(align));
    this.configureBindings();
  }


  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
    
  }

  public void initCommandInTeleop() {
    swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}