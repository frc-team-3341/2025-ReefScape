package frc.robot.commands.autoPaths;

import java.io.IOException;

import org.json.simple.parser.ParseException;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.AutoPath;

import frc.robot.subsystems.swerve.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightLine extends SequentialCommandGroup  {
  SwerveDriveTrain swerve;
  AutoPath autoPath;
  
  
  public StraightLine(SwerveDriveTrain swerve) throws FileVersionException, IOException, ParseException {
    this.swerve = swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // *TODO: tune the constants for shooting into the speaker from the left
      new AutoPath("StraightLine", this.swerve, false)
    );
  }
}