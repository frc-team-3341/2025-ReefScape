// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeepHang;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeepHangTeleop extends Command {
  /** Creates a new DeepHang. */
  DeepHang hang;
  Joystick joystick;

  public DeepHangTeleop(DeepHang hang, Joystick joystick) {
    this.hang = hang;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hang.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double up = joystick.getPOV();
    if(up < 15 || up > 345) {
      hang.set(0.3);
    } 
    if( up < 195 && up > 165) {
      hang.set(-0.3);
    }
    else {
      hang.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hang.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
