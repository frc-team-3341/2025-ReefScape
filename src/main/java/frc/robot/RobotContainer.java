package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.Elevator;
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
  //private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  // private final SwerveDriveTrain swerveDriveTrain = new SwerveDriveTrain(startpose,
  //         Constants.SwerveModuleIOConfig.module0,
  //         Constants.SwerveModuleIOConfig.module1,
  //         Constants.SwerveModuleIOConfig.module2,
  //         Constants.SwerveModuleIOConfig.module3);

  // private final SwerveTeleopCMD swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);

  // private final SwerveAutonomousCMD serveAutoCMD = new SwerveAutonomousCMD(this.swerveDriveTrain,
  //         Constants.allianceEnabled);
  // private TestFourModules allFour;
  // private CrabDrive crabDrive;
private final CommandXboxController elevatorController = new CommandXboxController(0);
  Elevator elevator = new Elevator(elevatorController);

  public RobotContainer() {
    // this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
    this.configureBindings();
    
  }


  private void configureBindings() {
    // elevatorController.b().onTrue(elevator.stopElevator());
    // elevatorController.povUp().whileTrue(elevator.upPovElevator());
    // elevatorController.povUp().whileFalse(elevator.stopElevator());
    
    // elevatorController.povDown().whileTrue(elevator.downPovElevator());
    // elevatorController.povDown().whileFalse(elevator.stopElevator());
    
    elevatorController.axisGreaterThan(5, 0.1).whileTrue(elevator.upElevator());
    elevatorController.axisLessThan(5, 0.1).and(elevatorController.axisGreaterThan(5, -0.1)).whileTrue(elevator.stopElevator());
    elevatorController.axisLessThan(5, -0.1).whileTrue(elevator.downElevator());
  }


  public Command getAutonomousCommand() {
    return null;
    // return serveAutoCMD;
    
  }

  public void initCommandInTeleop() {
    // swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}