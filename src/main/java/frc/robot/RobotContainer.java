package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(0,0), new Rotation2d());//new Pose2d(new Translation2d(8.2,4.2), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  // private final Joystick drivingXbox = new Joystick(0);
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
    
  private final Joystick drivingXbox = new Joystick(0);
  private final CommandXboxController mechXboxController = new CommandXboxController(1);

  private SwerveDriveTrain swerveDriveTrain;

  private SwerveTeleopCMD swerveTeleopCMD;

  private SwerveAutonomousCMD serveAutoCMD;

  private DeepHang deepHang;

  private CoralManipulator coralManipulator;

  private Elevator elevator;

  public RobotContainer() {
    createSwerve();
    //createDeepHang();
    //createCoralManipulator();
    //createElevator();
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


  private void configureBindings() {
    elevatorController.a().onTrue(elevator.homing());

        
    elevatorController.axisGreaterThan(5, 0.1).whileTrue(elevator.moveElevatorUp()); // If joystick is above 0.1, move up 
    elevatorController.axisLessThan(5, -0.1).whileTrue(elevator.moveElevatorDown()); // If joystick is below -0.1 move down

    Trigger elevStopB1 = elevatorController.axisLessThan(5, 0.1);
    //Elevator stop for bound 1 and 2 - between -0.1 and 0.1
    Trigger elevStopB2 = elevatorController.axisGreaterThan(5, -0.1);


    
    elevStopB1.and(elevStopB2).onTrue(elevator.stopElevator()); 
    // if the joystick changes from moving to being still (in bounds), then stop the elevator. It only toggles when the state changes, not repeatidly
    elevatorController.x().onTrue(elevator.setHeightL2()); //on button press
    elevatorController.y().onTrue(elevator.setHeightL3());
  }


  public Command getAutonomousCommand() {
    return null;
    // return serveAutoCMD;
    
  }

  public void initCommandInTeleop() {
    // swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}