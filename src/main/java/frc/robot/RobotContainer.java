package frc.robot;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.commands.targeting.Alignment;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.DeepHang;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.targeting.Vision;

public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(8.7, 4.0), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final CommandXboxController drivingXbox = new CommandXboxController(0);
  private final CommandJoystick mechJoystick = new CommandJoystick(1);

  private SwerveDriveTrain swerveDriveTrain;
  private SwerveTeleopCMD swerveTeleopCMD;
  
  private DeepHang deepHang;

  private CoralManipulator coralManipulator;

  private Elevator elevator;

  private Alignment align;
  private Vision vision;

  public RobotContainer() {

    // Starts recording to data log
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    createSwerve();
    createDeepHang();
    createCoralManipulator();
    createElevator();

    //Call this last since this creates the parallel command groups
    //and requires elevator and coral manipulator
    BALLASDHAKHSDHASDKJAS();
  }

  private void createSwerve() {
    //Swerve needs the vision make sure to create this first
    //Create swerveDriveTrain
    vision = new Vision(drivingXbox);
    swerveDriveTrain = new SwerveDriveTrain(startpose,
    Constants.SwerveModuleIOConfig.module0,
    Constants.SwerveModuleIOConfig.module1,
    Constants.SwerveModuleIOConfig.module2,
    Constants.SwerveModuleIOConfig.module3,
    vision, () -> {return drivingXbox.getLeftTriggerAxis();});
    
    //Create swerve commands here
    swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);


    //Set default swerve command to the basic drive command, not field orientated
    this.swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);

    //This requires the swerve subsystem make sure to create that first before creating this
    drivingXbox.x().onTrue(this.swerveDriveTrain.toggleFieldCentric());
    drivingXbox.y().onTrue(this.swerveDriveTrain.resetHeadingCommand());

    drivingXbox.leftTrigger(0.02).whileTrue(swerveDriveTrain.driveForward());

    // longAlignment = new LongitudinalAlignment(swerveDriveTrain, vision);
    align = new Alignment(swerveDriveTrain, vision);
    drivingXbox.a().toggleOnTrue(align);
    drivingXbox.leftBumper().onTrue(vision.setpointLeftHorizontal());
    drivingXbox.rightBumper().onTrue(vision.setpointRightHorizontal());
    drivingXbox.b().onTrue(vision.setpointZeroHorizontal());
  }

  private void createDeepHang() {
    deepHang = new DeepHang();
    
    mechJoystick.button(7).whileTrue(deepHang.fwd()).onFalse(deepHang.stop());

    mechJoystick.button(8).whileTrue(deepHang.rev()).onFalse(deepHang.stop());
  }

  private void createCoralManipulator() {
    coralManipulator = new CoralManipulator();

    mechJoystick.button(16).onTrue(coralManipulator.intakeCoral()).onFalse(coralManipulator.stopCoral());
    mechJoystick.button(18).onTrue(coralManipulator.releaseCoral()).onFalse(coralManipulator.stopCoral());
    //mechJoystick.button(6).toggleOnTrue(coralManipulator.movePivot());
    //mechJoystick.button(6).onTrue(coralManipulator.pivotIntake()); 
    //mechJoystick.button(7).onTrue(coralManipulator.pivotL4());
    //mechJoystick.button(8).onTrue(coralManipulator.pivotPlace());
    
    //mechJoystick.axisMagnitudeGreaterThan(7, 0.1).whileTrue(coralManipulator.movePivot());
    
    mechJoystick.button(6).onTrue(coralManipulator.toggleTeleop());
    mechJoystick.povUp().onTrue(coralManipulator.movePivotUp());
    mechJoystick.povDown().onTrue(coralManipulator.movePivotDown());
  }

  private void createElevator() {
    elevator = new Elevator(()->{
      return mechJoystick.getRawAxis(4);
    });

    elevator.setDefaultCommand(elevator.moveElevator());

    mechJoystick.button(5).onTrue(elevator.toggleTeleop());

    //Creates a new trigger for when the rev limit is pressed.
    Trigger elevatorHoming = new Trigger(() -> {
      return elevator.isREVLimit();
    });
    //When the rev limit switch is pressed, reset the encoders.
    //This approach is better than having it in periodic since
    //when the rev limit is pressed an interrupt is sent to reset the encoders
    //instead of constantly checking in periodic if the rev limit switch is pressed
    elevatorHoming.onTrue(elevator.stopElevator());
    elevatorHoming.onFalse(elevator.resetEncoder());
  }

  private void BALLASDHAKHSDHASDKJAS() {
    ParallelCommandGroup gotoL1 = new ParallelCommandGroup(elevator.homeElevatorDown(), coralManipulator.pivotPlace());
    ParallelCommandGroup gotoL2 = new ParallelCommandGroup(elevator.setHeightL2(), coralManipulator.pivotPlace());
    ParallelCommandGroup gotoL3 = new ParallelCommandGroup(elevator.setHeightL3(), coralManipulator.pivotPlace());
    ParallelCommandGroup gotoL4 = new ParallelCommandGroup(elevator.setHeightL4(), coralManipulator.pivotL4());
    //Change this to use the home command if the homeEleavtorDown doesn't work
    ParallelCommandGroup gotoIntake = new ParallelCommandGroup(elevator.homeElevatorDown(), coralManipulator.pivotIntake());
    mechJoystick.button(17).and(()->!coralManipulator.probablyHasCoral).onTrue(gotoIntake);
    mechJoystick.button(1 ).onTrue(gotoL4);
    mechJoystick.button(2 ).onTrue(gotoL3);
    mechJoystick.button(3 ).onTrue(gotoL2);
    mechJoystick.button(4 ).onTrue(gotoL1);

    //These are for auto. Triggers that happen during auto paths to execute commands
    new EventTrigger("Go to L4").onTrue(new ParallelCommandGroup(elevator.setHeightL4(), coralManipulator.pivotL4()));
    new EventTrigger("Score Coral").onTrue(new SequentialCommandGroup(new WaitCommand(1), coralManipulator.releaseCoral(), new WaitCommand(1), coralManipulator.stopCoral()));
    new EventTrigger("Home Elevator and Coral").onTrue(gotoIntake);
    new EventTrigger("Get Coral").onTrue(new SequentialCommandGroup(coralManipulator.intakeCoral(), new WaitCommand(1), coralManipulator.stopCoral()));
  
    SmartDashboard.putData("Homing", new ParallelCommandGroup(elevator.homeElevatorDown(), coralManipulator.pivotDown()));
    SmartDashboard.putData("togglePoseEst", new SequentialCommandGroup(vision.togglePoseEst()));
  }

  public void togglePoseEst() {
    //swerveDriveTrain.togglePoseEst();
    vision.togglePoseEst();
  }

  public Command getAutonomousCommand() {
    return swerveDriveTrain.getAutonomousCommand();
  }

  public void initCommandInTeleop() {
    elevator.homeElevatorDown();
    //swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}