package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import edu.wpi.first.wpilibj.Joystick;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(0,0), new Rotation2d());//new Pose2d(new Translation2d(8.2,4.2), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  //private final CommandXboxController mechXboxController = new CommandXboxController(1);
  private final Joystick mechXboxController = new Joystick(1);

  private SwerveDriveTrain swerveDriveTrain;

  private SwerveTeleopCMD swerveTeleopCMD;

  private SwerveAutonomousCMD serveAutoCMD;

  private DeepHang deepHang;

  private CoralManipulator coralManipulator;

  private Elevator elevator;

  public RobotContainer() {
    createSwerve();
    createDeepHang();
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
    
    //deepHang.setDefaultCommand(deepHang.stop());
    //mechXboxController.povUp().whileTrue(deepHang.fwd());
    //mechXboxController.povUp().onFalse(deepHang.stop());

    //mechXboxController.povDown().whileTrue(deepHang.rev());
    //mechXboxController.povDown().onFalse(deepHang.stop());
    if(mechXboxController.getRawAxis(1) > 0.1){
      deepHang.fwd();
    } else if(mechXboxController.getRawAxis(1) < -0.1){
      deepHang.rev();
    } else {
      deepHang.stop();
    }

    if(mechXboxController.getRawButton(20)) {
      deepHang.homing();
    }
    //mechXboxController.x().onTrue(deepHang.homing());
  }

  private void createCoralManipulator() {
  /* 
    coralManipulator = new CoralManipulator(()->{
      return mechXboxController.getLeftY();
    });
  */
    //coralManipulator.setDefaultCommand(coralManipulator.stopCoral());
    /** 
    mechXboxController.axisGreaterThan(2, 0).whileTrue(coralManipulator.pivotStop());
    mechXboxController.x().whileTrue(coralManipulator.intakeCoral()).onFalse(coralManipulator.stopCoral());
    mechXboxController.b().whileTrue(coralManipulator.releaseCoral()).onFalse(coralManipulator.stopCoral());
    
    mechXboxController.povUp().onTrue(coralManipulator.pivotL4());
    mechXboxController.povDown().onTrue(coralManipulator.pivotDown());
    mechXboxController.povRight().onTrue(coralManipulator.pivotIntake());
  

    mechXboxController.axisGreaterThan(1, 0.1).whileTrue(coralManipulator.movePivot());
    mechXboxController.axisLessThan(1, -0.1).whileTrue(coralManipulator.movePivot());

    Trigger coralStopB1 = mechXboxController.axisLessThan(1, 0.1);
    Trigger coralStopB2 = mechXboxController.axisGreaterThan(1, -0.1);
    
    coralStopB1.and(coralStopB2).onTrue(coralManipulator.pivotStop()); 
    */
  /*
    mechXboxController.a().onTrue(coralManipulator.intakeCoral()).onFalse(coralManipulator.stopCoral());
    mechXboxController.b().onTrue(coralManipulator.releaseCoral()).onFalse(coralManipulator.stopCoral());
  */
    }

  private void createElevator() {
  /*
    elevator = new Elevator(()->{
      return mechXboxController.getRightY();
    });
  */
    //mechXboxController.leftBumper().onTrue(elevator.homing());
    //mechXboxController.rightBumper().onTrue(elevator.stopElevator());
    
    // if the joystick changes from moving to being still (in bounds), then stop the elevator. It only toggles when the state changes, not repeatidly
    //mechXboxController.a().onTrue(elevator.setHeightL1()); //on button press
    //mechXboxController.b().onTrue(elevator.setHeightL2()); //on button press
    //mechXboxController.x().onTrue(elevator.setHeightL3()); //on button press
    //mechXboxController.y().onTrue(elevator.setHeightL4()); //on button press
    //We apply the deadband inside this function
  /*
    mechXboxController.y().toggleOnTrue(elevator.moveElevator());

    //Should change this stop to stall so the elevator can hold its position
    mechXboxController.y().toggleOnFalse(elevator.stopElevator());
  */
  }

  public Command getAutonomousCommand() {
    return null;
    
  }

  public void initCommandInTeleop() {
    //swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}