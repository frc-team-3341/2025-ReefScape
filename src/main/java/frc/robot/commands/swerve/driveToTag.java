// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class driveToTag extends Command {
  private SwerveDriveTrain swerve;

  /** Creates a new driveToTag. */
  public driveToTag(SwerveDriveTrain swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if the Limelight sees the tag 
    double tx = LimelightHelpers.getTX("limelight-shahzhu");
    double ty = LimelightHelpers.getTY("limelight-shahzhu");

    // If Limelight doesn't see any target, don't move
    if(Math.abs(tx) < 0.5 && Math.abs(ty) < 0.5){
      swerve.stopMotors();
      return;
    }

    double avgDistance = -1;
    
    // Calculate the angular velocity based on Limelight aiming (proportional control)
    double rot = limelight_aim_proportional();

    // Calculate the forward speed based on Limelight ranging (proportional control)
    double xSpeed = limelight_range_proportional();

    // You can choose whether to drive field-relative or not (typically false for autonomous)
    boolean fieldRelative = false;
    
    // Drive the swerve drive system
    //Translation2d(0 ,0) - distances in the (x, y) direction the robot needs to travel to get to position
    //Second Parameter is the rotation needed - right now we are testing just 90 degrees
    //we will use the Tx from limelight helpers class for rotation
    //We will use trig to calculate the translation distances but we will need Tx and some data testing to do that
    double[] cameraPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-shahzhu");
    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate("limelight-shahzhu", "botpose");
    if (poseEstimate != null) {
        avgDistance = poseEstimate.avgTagDist;
        //System.out.println("Average Distance from Tag: " + avgDistance);
    } else {
        //System.out.println("No pose estimate available.");
    }

    // Stop the robot if it's within 0.45-0.5 meters of the target
    double distanceThreshold1 = 0;
    double distanceThreshold2 = 0.3;
    if(avgDistance >= distanceThreshold1 && avgDistance <= distanceThreshold2){
      swerve.stopMotors();
      return;
    }

    // if(swerve.getPoseFromEstimator().getX() > 0.5 ){
    //   swerve.stopMotors();
    //   return;
    // }
    ShuffleboardTab tab = Shuffleboard.getTab("Pose (x, y, z)");
      Shuffleboard.selectTab("Pose (x, y, z)");
      SmartDashboard.putNumber("x", cameraPoseTargetSpace[0]);
      SmartDashboard.putNumber("y", cameraPoseTargetSpace[1]);
      SmartDashboard.putNumber("z", cameraPoseTargetSpace[2]);
      SmartDashboard.putNumber("TX?", LimelightHelpers.getTX("limelight-shahzhu"));
      SmartDashboard.putNumber("AvgDistanceFromLimelight", avgDistance);
      SmartDashboard.updateValues();
      //LimelightHelpers.getTX("limelight-shahzhu")
     swerve.drive(new Translation2d(cameraPoseTargetSpace[3] * -1 * 0.02, cameraPoseTargetSpace[0] * -2), 0.002 * LimelightHelpers.getTX("limelight-shahzhu"), false, false);
    //swerve.drive(new Translation2d(-0.5, -.5), 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return false;
  }

  // Simple proportional turning control with Limelight
    private double limelight_aim_proportional() {
        double kP = .035;  // Proportional constant
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight-shahzhu") * kP;

        // Convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.SwerveConstants.maxChassisAngularVelocity;

        // Invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // Simple proportional ranging control with Limelight's "ty" value
    private double limelight_range_proportional() {
        double kP = .1;  // Proportional constant
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight-shahzhu") * kP;

        targetingForwardSpeed *= Constants.SwerveConstants.maxChassisTranslationalSpeed;
        targetingForwardSpeed *= -1.0;

        return targetingForwardSpeed;
    }
}

