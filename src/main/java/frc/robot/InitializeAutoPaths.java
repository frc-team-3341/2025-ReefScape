// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.commands.autoPaths.StraightLine;

/** Add your docs here. */
public class InitializeAutoPaths {
    private final SwerveDriveTrain swerve;

    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
    
    // Plays:
    private StraightLine straightLine;
    //1st part of auto phase path
    private StartingPos_CoralLoc4 startingPosCoralLoc4;
    //2nd part of auto phase path
    private CoralLoc4_CStation coralLoc4CStation;
    //3rd part of auto phase path
    private CStation_CoralLoc2 cStationCoralLoc2;

    RobotConfig config;

    public InitializeAutoPaths(SwerveDriveTrain swerve) {
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
        this.swerve = swerve;
        AutoBuilder.configure(
        this.swerve::getPoseFromEstimator, // Robot pose supplier
        this.swerve::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this.swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this.swerve::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants                                                      
        ),
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this.swerve // Reference to this subsystem to set requirements
        );

         // PLAYS:
         
        try {
          straightLine = new StraightLine(this.swerve);
          
          autoCommandChooser.setDefaultOption("straightLine", straightLine);
          //1st part of auto path phase
          autoCommandChooser.addOption("startingPosCoralLoc4", startingPosCoralLoc4);
          //2nd part of auto path phase
          autoCommandChooser.addOption("coralLoc4CStation", coralLoc4CStation);
          //3rd part of auto path phase
          autoCommandChooser.addOption("cStationCoralLoc2", cStationCoralLoc2);

          SmartDashboard.putData(autoCommandChooser);

        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
       
    }

    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected(); //Gives selected command to RobotContainer
    }
}
