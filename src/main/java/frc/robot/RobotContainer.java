package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.DeepHang;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.InitializeAutoPaths;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(1.8, 6), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  private final CommandXboxController mechXboxController = new CommandXboxController(1);

  private SwerveDriveTrain swerveDriveTrain;

  private SwerveTeleopCMD swerveTeleopCMD;

  private SwerveAutonomousCMD serveAutoCMD;

  // Auto Trajectories
  private InitializeAutoPaths autoPaths;
  
  private DeepHang deepHang;

  private CoralManipulator coralManipulator;

  private Elevator elevator;

  private Vision vision;



  public RobotContainer() {
    createSwerve();
    //createDeepHang();
    //createCoralManipulator();
    //createElevator();
    this.configureAuto(); // make sure to call after swerve is configured
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
    vision = new Vision();
  }

  private void createSwerve() {
    //Create swerveDriveTrain
    swerveDriveTrain = new SwerveDriveTrain(startpose,
    Constants.SwerveModuleIOConfig.module0,
    Constants.SwerveModuleIOConfig.module1,
    Constants.SwerveModuleIOConfig.module2,
    Constants.SwerveModuleIOConfig.module3);
    
    //Create swerve commands here
    swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);
    serveAutoCMD = new SwerveAutonomousCMD(this.swerveDriveTrain, Constants.allianceEnabled);

    //Set default swerve command to the basic drive command, not field orientated
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }

  private void createDeepHang() {
    deepHang = new DeepHang();
    
    mechXboxController.povUp().whileTrue(deepHang.fwd());
    mechXboxController.povUp().whileFalse(deepHang.stop());

    mechXboxController.povDown().whileTrue(deepHang.rev());
    mechXboxController.povDown().whileFalse(deepHang.stop());
  }

  private void createCoralManipulator() {
    coralManipulator = new CoralManipulator(mechXboxController);

    mechXboxController.x().onTrue(coralManipulator.stopCoral());
    mechXboxController.y().onTrue(coralManipulator.intakeCoral());
    mechXboxController.b().onTrue(coralManipulator.releaseCoral());
    mechXboxController.axisGreaterThan(2, 0).whileTrue(coralManipulator.pivotStop());
    mechXboxController.povUp().onTrue(coralManipulator.pivotUp());
    mechXboxController.povDown().onTrue(coralManipulator.pivotDown());
  }

  private void createElevator() {
    elevator = new Elevator(() -> mechXboxController.getRightY());
    mechXboxController.a().onTrue(elevator.setHeightL1());
    /** 

    mechXboxController.a().onTrue(elevator.homing());
        
    mechXboxController.axisGreaterThan(5, 0.1).whileTrue(elevator.moveElevatorUp()); // If joystick is above 0.1, move up 
    mechXboxController.axisLessThan(5, -0.1).whileTrue(elevator.moveElevatorDown()); // If joystick is below -0.1 move down

    // mechXboxController.a().onTrue(elevator.homing());
    
    //directions are flipped
    mechXboxController.axisGreaterThan(5, 0.1).onTrue(elevator.moveElevatorDown()); // If joystick is above 0.1, move down 
    mechXboxController.axisLessThan(5, -0.1).onTrue(elevator.moveElevatorUp()); // If joystick is below -0.1 move up
    
    Trigger elevStopB1 = mechXboxController.axisLessThan(5, 0.1);
    //Elevator stop for bound 1 and 2 - between -0.1 and 0.1
    Trigger elevStopB2 = mechXboxController.axisGreaterThan(5, -0.1);
    
    elevStopB1.and(elevStopB2).onTrue(elevator.stopElevator());  // It needs to hold position not completely stop
    // if the joystick changes from moving to being still (in bounds), then stop the elevator. It only toggles when the state changes, not repeatidly
    mechXboxController.x().onTrue(elevator.setHeightL4()); //on button press
    */
  }

  private void configureAuto(){
    autoPaths = new InitializeAutoPaths(this.swerveDriveTrain);
  }

  public Command getAutonomousCommand() {
    return autoPaths.getAutonomousCommand();
  }

  public void initCommandInTeleop() {
    swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}