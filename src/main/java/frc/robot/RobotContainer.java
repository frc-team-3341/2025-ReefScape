package frc.robot;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.commands.targeting.Alignment;
import frc.robot.commands.targeting.LongitudinalAlignment;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.DeepHang;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.targeting.Vision;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(1.8, 6), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final CommandXboxController drivingXbox = new CommandXboxController(0);
  private final CommandJoystick mechJoystick = new CommandJoystick(1);  // New joystick 
  //private final CommandXboxController mechController = new CommandXboxController(2);


  private SwerveDriveTrain swerveDriveTrain;

  private SwerveTeleopCMD swerveTeleopCMD;

  // Auto Trajectories
  private InitializeAutoPaths autoPaths;
  
  private DeepHang deepHang;

  private CoralManipulator coralManipulator;

  private Elevator elevator;

  private Alignment align;
  private LongitudinalAlignment longAlignment;
  private Vision vision;

  

  public RobotContainer() {
    createSwerve();
    //createDeepHang();
    createCoralManipulator();
    //createElevator();
    mechJoystick.button(7).onTrue(new SequentialCommandGroup(coralManipulator.pivotIntake(), coralManipulator.intakeCoral(),  new WaitCommand(1), coralManipulator.stopCoral()));
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }

  private void createSwerve() {
    //Swerve needs the vision make sure to create this first
    //Create swerveDriveTrain
    vision = new Vision();
    swerveDriveTrain = new SwerveDriveTrain(startpose,
    Constants.SwerveModuleIOConfig.module0,
    Constants.SwerveModuleIOConfig.module1,
    Constants.SwerveModuleIOConfig.module2,
    Constants.SwerveModuleIOConfig.module3,
    vision);
    
    //Create swerve commands here
    swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);

    //Set default swerve command to the basic drive command, not field orientated
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);

    //This requires the swerve subsystem make sure to create that first before creating this
    autoPaths = new InitializeAutoPaths(this.swerveDriveTrain);
    drivingXbox.button(3).onTrue(this.swerveDriveTrain.toggleFieldCentric());
    drivingXbox.button(4).onTrue(this.swerveDriveTrain.resetHeadingCommand());

    longAlignment = new LongitudinalAlignment(swerveDriveTrain, vision);
    align = new Alignment(swerveDriveTrain, vision);
    drivingXbox.a().toggleOnTrue(new SequentialCommandGroup(align, longAlignment));
  }

  private void createDeepHang() {
    deepHang = new DeepHang();
    
    mechJoystick.axisGreaterThan(1, 0).whileTrue(deepHang.fwd());
    mechJoystick.axisGreaterThan(1, 0).onFalse(deepHang.stop());

    mechJoystick.axisLessThan(1, -0.1).whileTrue(deepHang.rev());
    mechJoystick.axisLessThan(1, -0.1).onFalse(deepHang.stop());
  }

  private void createCoralManipulator() {
    coralManipulator = new CoralManipulator(() -> {
      return mechJoystick.getRawAxis(7);
    });

    mechJoystick.button(16).onTrue(coralManipulator.intakeCoral()).onFalse(coralManipulator.stopCoral());
    mechJoystick.button(18).onTrue(coralManipulator.releaseCoral()).onFalse(coralManipulator.stopCoral());
    
    //TODO: Figure out what this button should be
    mechJoystick.button(17).toggleOnTrue(coralManipulator.movePivot());  
    mechJoystick.button(6).onTrue(coralManipulator.pivotIntake()); 
    mechJoystick.button(7).onTrue(coralManipulator.pivotL4());
    mechJoystick.button(8).onTrue(coralManipulator.pivotPlace());
    
    mechJoystick.axisMagnitudeGreaterThan(7, 0.1).whileTrue(coralManipulator.movePivot());
    
  }

  private void createElevator() {
    elevator = new Elevator(()->{
      return mechJoystick.getRawAxis(5);
    });

    //mechJoystick.axisMagnitudeGreaterThan(5, 0.1).whileTrue(elevator.moveElevator());
    mechJoystick.button(17).toggleOnTrue(elevator.moveElevator());

    mechJoystick.button(1).onTrue(elevator.setHeightL4());
    mechJoystick.button(2).onTrue(elevator.setHeightL3());
    mechJoystick.button(3).onTrue(elevator.setHeightL2());

    //Creates a new trigger for when the rev limit is pressed.
    Trigger elevatorHoming = new Trigger(() -> {
      return elevator.isREVLimit();
    });
    //When the rev limit switch is pressed, reset the encoders.
    //This approach is better than having it in periodic since
    //when the rev limit is pressed an interrupt is sent to reset the encoders
    //instead of constantly checking in periodic if the rev limit switch is pressed
    elevatorHoming.onTrue(elevator.resetEncoder());
  }

  public void CreateAutoCommands(){
      new EventTrigger("Score Coral L4").onTrue(new SequentialCommandGroup(new ParallelCommandGroup(elevator.setHeightL2(), coralManipulator.pivotL4()), new WaitCommand(1), elevator.setHeightL4(), coralManipulator.releaseCoral(), new WaitCommand(1), coralManipulator.stopCoral()));
      new EventTrigger("Home Elevator and Coral").onTrue(new ParallelCommandGroup(elevator.homeElevatorDown(), coralManipulator.homeDown()));
      new EventTrigger("Get Coral").onTrue(new SequentialCommandGroup(coralManipulator.pivotIntake(), coralManipulator.intakeCoral(), coralManipulator.stopCoral()));
      // Testing Purposes:
      new EventTrigger("Test Coral Intake").onTrue(new SequentialCommandGroup(coralManipulator.pivotIntake(), coralManipulator.intakeCoral(), new WaitCommand(2), coralManipulator.stopCoral()));
      new EventTrigger("Test Coral Outake").onTrue(new SequentialCommandGroup(coralManipulator.pivotL4(), coralManipulator.releaseCoral(), new WaitCommand(2), coralManipulator.stopCoral()));
      new EventTrigger("Test Elevator").onTrue(new SequentialCommandGroup(elevator.setHeightL2()));
  }

  public Command getAutonomousCommand() {
    return autoPaths.getAutonomousCommand();
  }

  public void initCommandInTeleop() {
    //swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}