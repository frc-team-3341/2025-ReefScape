package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.SwerveAutonomousCMD;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;


public class RobotContainer {

  // ---------------------- START OF CONFIG SECTION --------------------------

  // Defines starting pose of robot
  // TODO - Please remove this in future if developing for AprilTags
  Pose2d startpose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
  // add start pose if needed
  // ---------------------- END OF CONFIG SECTION --------------------------

  // Xbox + an additional one for PC use
  private final Joystick drivingXbox = new Joystick(0);
  XboxController xbox = new XboxController(1);
  BallShooter shooter = new BallShooter();
  EventLoop m_loop = new EventLoop();
  //private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

  private SwerveDriveTrain swerveDriveTrain;

  private SwerveTeleopCMD swerveTeleopCMD;

  private SwerveAutonomousCMD serveAutoCMD;
  // private TestFourModules allFour;
  // private CrabDrive crabDrive;



  public RobotContainer() {
    
    constructSwerve();
    this.configureBindings();
  }

  private void constructSwerve() {
    if (Constants.isSim) {
      swerveDriveTrain = new SwerveDriveTrain(startpose,
      new SwerveModuleIOSim(0), 
      new SwerveModuleIOSim(1),
      new SwerveModuleIOSim(2),
      new SwerveModuleIOSim(3));

    } else {
      swerveDriveTrain = new SwerveDriveTrain(startpose,
      Constants.SwerveModuleIOConfig.module0,
      Constants.SwerveModuleIOConfig.module1,
      Constants.SwerveModuleIOConfig.module2,
      Constants.SwerveModuleIOConfig.module3);
    }
    swerveTeleopCMD = new SwerveTeleopCMD(this.swerveDriveTrain, this.drivingXbox);
    serveAutoCMD = new SwerveAutonomousCMD(this.swerveDriveTrain, Constants.allianceEnabled);

    swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return serveAutoCMD;
    
  }

  public void initCommandInTeleop() {
    swerveDriveTrain.setDefaultCommand(swerveTeleopCMD);
  }

  private void configureLimeLight() {
    //Port forward the limelight ports so that we can access limelight dashboard during competetion when tethered
    for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
            //PortForwarder.add(port + 10, "limelight-left.local", port);
        }
  }
}