package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.CoralManipulator;
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
  //private final Joystick drivingXbox = new Joystick(0);
  //private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  // private final SwerveDriveTrain swerveDriveTrain = new SwerveDriveTrain(startpose,
  //         Constants.SwerveModuleIOConfig.module0,
  //         Constants.SwerveModuleIOConfig.module1,
  //         Constants.SwerveModuleIOConfig.module2,
  //         Constants.SwerveModuleIOConfig.module3);

  // private final SwerveTeleopCMD swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);

  // //private final SwerveAutonomousCMD serveAutoCMD = new SwerveAutonomousCMD(this.swerveDriveTrain,
  //         Constants.allianceEnabled);
  // // private TestFourModules allFour;
  // // private CrabDrive crabDrive;


private final CommandXboxController xboxController = new CommandXboxController(0);


  CoralManipulator coralManipulator = new CoralManipulator();

  public RobotContainer() {
    //this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
    this.configureBindings();
  }

  private void configureBindings() {
    xboxController.button(3).onTrue(coralManipulator.stopCoral());
    xboxController.button(1).onTrue(coralManipulator.intakeCoral());
    xboxController.button(2).onTrue(coralManipulator.releaseCoral());
    xboxController.axisGreaterThan(2, 0).whileTrue(coralManipulator.pivotStop());
    xboxController.povUp().onTrue(coralManipulator.pivotUp());
    xboxController.povDown().onTrue(coralManipulator.pivotDown());
    xboxController.button(0).onTrue(coralManipulator.spinPivot10());
  }

  public Command getAutonomousCommand() {
     return null;
    //return serveAutoCMD;


  }

  public void initCommandInTeleop() {
    //swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}