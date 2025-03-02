package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Homing;
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
  private final CommandJoystick mechJoystick = new CommandJoystick(1);  // New joystick 
  private final CommandXboxController mechController = new CommandXboxController(2);


  private SwerveDriveTrain swerveDriveTrain;

  private SwerveTeleopCMD swerveTeleopCMD;

  private SwerveAutonomousCMD serveAutoCMD;

  private DeepHang deepHang;

  private CoralManipulator coralManipulator;

  private Elevator elevator;

  public RobotContainer() {
    // createSwerve();
    //createDeepHang();
    createCoralManipulator();
    createElevator();
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

  // private void createDeepHang() {
  //   deepHang = new DeepHang();
    
    
  //   mechController.axisGreaterThan(1, 0, false).whileTrue(deepHang.fwd());
  //   mechController.axisGreaterThan(1, 0, false).onFalse(deepHang.stop());

  //   mechController.axisLessThan(1, -0.1, true).whileTrue(deepHang.rev());
  //   mechController.axisLessThan(1, -0.1, true).onFalse(deepHang.stop());
  // }

  private void createCoralManipulator() {
    coralManipulator = new CoralManipulator(() -> {
      return mechController.getLeftY();
    });

    // Intake (Button 16) and Release (Button 18)
    mechJoystick.button(16).whileTrue(coralManipulator.intakeCoral()).onFalse(coralManipulator.stopCoral());
    mechJoystick.button(18).whileTrue(coralManipulator.releaseCoral()).onFalse(coralManipulator.stopCoral());

    mechJoystick.button(7).onTrue(coralManipulator.pivotIntake()); // TEMP
    mechJoystick.button(8).onTrue(coralManipulator.pivotPlace()); //TEMP
           
    mechController.axisGreaterThan(1, 0.1).whileTrue(coralManipulator.movePivot()); 
    mechController.axisLessThan(1, -0.1).whileTrue(coralManipulator.movePivot()); 

    // mechController.axisMagnitudeGreaterThan(1, 0.1).whileTrue(coralManipulator.movePivot()).onFalse(coralManipulator.pivotStop()e);

    Trigger coralStopB1 = mechController.axisLessThan(5, 0.1);
    Trigger coralStopB2 = mechController.axisGreaterThan(5, -0.1);
    
    coralStopB1.and(coralStopB2).onTrue(coralManipulator.pivotStop()); 
    
  }

  private void createElevator() {
    elevator = new Elevator(()->{
      return mechJoystick.getRawAxis(5); //TESTING
    });

    Homing home = new Homing(elevator);
            
    mechController.axisGreaterThan(5, 0.1).whileTrue(elevator.moveElevator()); // If joystick is above 0.1, move up 
    mechController.axisLessThan(5, -0.1).whileTrue(elevator.moveElevator()); // If joystick is below -0.1 move down

    Trigger elevStopB1 = mechController.axisLessThan(5, 0.1);
    //Elevator stop for bound 1 and 2 - between -0.1 and 0.1
    Trigger elevStopB2 = mechController.axisGreaterThan(5, -0.1);
    
    elevStopB1.and(elevStopB2).onTrue(elevator.stopElevator());  // It needs to hold position not completely stop

    // if the joystick changes from moving to being still (in bounds), then stop the elevator. It only toggles when the state changes, not repeatidly
    
    // mechJoystick.button(0).onTrue(elevator.stallElevator()); //TEMPORARY
    // mechJoystick.button(0).onTrue(elevator.setHeightL2());
    // mechJoystick.button(0).onTrue(elevator.setHeightL3());
    // mechJoystick.button(0).onTrue(elevator.setHeightL4());

    mechJoystick.button(6).onTrue(home); 
    // mechJoystick.button(0).onTrue(elevator.stopElevator()); //TEMP
  }

  public Command getAutonomousCommand() {
    return null;
    
  }

  public void initCommandInTeleop() {
    //swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }
}