  package frc.robot.commands.swerve;
// Copyright (c) FIRST and other WPILib contributors.

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class AutoPath extends SequentialCommandGroup {

  SwerveDriveTrain swerve;
  boolean firstPath;
  Pose2d initialPose;

  /**
   * Creates a new SwerveAuto.
   * 
   * @param pathName Name of path in RIO's data folder
   * @param swerve   SwerveDrive subsystem
   * @throws ParseException 
   * @throws IOException 
   * @throws FileVersionException 
   */
  public AutoPath(String pathName, SwerveDriveTrain swerve, boolean firstPath) throws FileVersionException, IOException, ParseException {
    this.swerve = swerve;
    
    // Load path from 2025 PathPlannerLib
    
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      // Will automagically re-display the path every time teleop is started
      // Set field's trajectory to the trajectory of the path
      this.swerve.getField().getObject("traj").setPoses(poses);
    });

    var swerveAuto = AutoBuilder.followPath(path);

    // Setting voltage to 0 is necessary in order to stop robot
    addCommands(swerveAuto.beforeStarting(() -> {
     // Need to initialize the starting pose in here
      //Possible ways to get the start pose of the path
      Optional<Pose2d> optionalPose2d = path.getStartingHolonomicPose();
      SmartDashboard.putBoolean("resetpose", false);
      if(firstPath == true){
        if (optionalPose2d.isPresent()) {
          SmartDashboard.putBoolean("resetpose", true);
          swerve.resetPose(optionalPose2d.get());
        }
      }
      else {
        //we want to do nothing if it's not the first path that's being used
      }
    }).finallyDo(() -> {
      swerve.setModulesPositions(0, 0);
      swerve.setModuleVoltages(0, 0);
    }));
  }
  
}
