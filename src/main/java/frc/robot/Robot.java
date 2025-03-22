// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private static Optional<Alliance> alliance = DriverStation.getAlliance();


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    //Pathplanner recommends running this command to get everything ready
    //FollowPathCommand.warmupCommand().schedule();
    // CameraServer.startAutomaticCapture();
    // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    // MjpegServer server = new MjpegServer("server_USB_Camera_0", 1181);
    // server.setSource(usbCamera);
    // CvSink sink = CameraServer.getVideo();
    SmartDashboard.putBoolean("Is Running", false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    SmartDashboard.putBoolean("Is Running", !m_autonomousCommand.isFinished());
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
  
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //Uncomment this if you want to disable pose est when running teleop
    //This will allow allignment to work
    //m_robotContainer.togglePoseEst();
    
    alliance = DriverStation.getAlliance();
    
    m_robotContainer.initCommandInTeleop();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  
  public static Optional<Alliance> getAlliance() {
    return alliance;
  }
}
