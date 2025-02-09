package frc.robot.commands.swerve.targeting;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.swerve.targeting.Vision;

public class Alignment extends Command{
    SwerveTeleopCMD teleop;
    XboxController xbox = new XboxController(0);
    SwerveDriveTrain swerve;
    Vision vision;
    Joystick joy = new Joystick(0);

    double rotDirection;
    double horizDirection;
    double lHorizDirection;
    double lRotDirection;
    boolean state;

    PIDController pid = new PIDController(1, 0, 0.05);

    public Alignment(SwerveDriveTrain swerve, Vision vision) {
        this.vision = vision;
        this.swerve = swerve;
        teleop = new SwerveTeleopCMD(swerve, joy);

        addRequirements(this.vision, this.swerve);
        pid.setTolerance(0.02);
    }

    @Override
    public void initialize() {
        swerve.drive(new Translation2d(0,0), 0, false, false);
    }

    // @Override
    // public void execute() {

    //   vision.switchHorizontalSetpoint();

    //   if (vision.targetDetected() && (!vision.rotationalAtSetpoint() || !vision.horizontalAtSetpoint()) && !vision.joystickHeld()) { 
    //     rotDirection = vision.getRotationalDirection();
    //     horizDirection = vision.getHorizontalDirection();

    //     swerve.drive(new Translation2d(0, 0.3*horizDirection), 0.3*rotDirection, false, false);
    //   }
    //   else if (!vision.targetDetected() && (vision.getLastHorizPosition() != 0 || vision.getLastRotAngle() != 0)) {

    //     if (vision.getLastHorizPosition() < 0) {
    //         lHorizDirection = -1;
    //     }

    //     else if (vision.getLastHorizPosition() > 0) {
    //         lHorizDirection = 1;
    //     }

    //     if (vision.getLastRotAngle() < 178) {
    //         lRotDirection = 1;
    //     }

    //     else if (vision.getLastRotAngle() > 182) {
    //         lRotDirection = -1;
    //     }
        
    //     swerve.drive(new Translation2d(0, 0.3*lHorizDirection), 0.3*lRotDirection, false, false);
    //   }
    //   if (vision.joystickHeld()) {
    //     // if(vision.getHorizontalDirection() > 0) {
    //     //     if(vision.getRotationalDirection() < 0) {
    //     //         swerve.drive(new Translation2d(0, -xbox.getRawAxis(0)*0.8*horizDirection), -xbox.getRawAxis(4)*0.9*rotDirection, false, false);
    //     //     }
    //     //     else if(vision.getRotationalDirection() > 0) {
    //     //         swerve.drive(new Translation2d(0, -xbox.getRawAxis(0)*0.8*horizDirection), xbox.getRawAxis(4)*0.9*rotDirection, false, false);
    //     //     }
    //     //     swerve.drive(new Translation2d(0, -xbox.getRawAxis(0)*0.8*horizDirection), xbox.getRawAxis(4)*0.9, false, false);
    //     // }
    //     // if (vision.getHorizontalDirection() < 0) {
    //     //     if(vision.getRotationalDirection() < 0) {
    //     //         swerve.drive(new Translation2d(0, xbox.getRawAxis(0)*0.8*horizDirection), -xbox.getRawAxis(4)*0.9*rotDirection, false, false);
    //     //     }
    //     //     else if(vision.getRotationalDirection() > 0) {
    //     //         swerve.drive(new Translation2d(0, xbox.getRawAxis(0)*0.8*horizDirection), xbox.getRawAxis(4)*0.9*rotDirection, false, false);
    //     //     }
    //     //     swerve.drive(new Translation2d(0, xbox.getRawAxis(0)*0.8*horizDirection), xbox.getRawAxis(4)*0.9, false, false);
    //     // }
    //     // swerve.drive(new Translation2d(0, -xbox.getRawAxis(0)*0.8), xbox.getRawAxis(4)*0.9*rotDirection, false, false);
    //     swerve.drive(new Translation2d(-1.5*xbox.getRawAxis(1), -1.5*xbox.getRawAxis(0)), -1.5*xbox.getRawAxis(4), false, false);
    //   }
    // }

    @Override
    public void execute() {

      vision.switchHorizontalSetpoint();

      if (vision.targetDetected() && (!vision.rotationalAtSetpoint() || !vision.horizontalAtSetpoint()) && !vision.joystickHeld()) { 
        rotDirection = vision.getRotationalDirection();
        horizDirection = -pid.calculate(vision.getHorizontalDisplacement(), 0);

        System.out.println("pid shiz: " + horizDirection);
        System.out.println(vision.getHorizontalDirection());

        swerve.drive(new Translation2d(0, 3.441*horizDirection), 0.5*rotDirection, false, false);
      }
      else if (!vision.targetDetected() && (vision.getLastHorizPosition() != 0 || vision.getLastRotAngle() != 0)) {

        if (vision.getLastHorizPosition() < 0) {
            lHorizDirection = -1;
        }

        else if (vision.getLastHorizPosition() > 0) {
            lHorizDirection = 1;
        }

        if (vision.getLastRotAngle() < 178) {
            lRotDirection = 1;
        }

        else if (vision.getLastRotAngle() > 182) {
            lRotDirection = -1;
        }
        
        swerve.drive(new Translation2d(0, 0.3*lHorizDirection), 0.3*lRotDirection, false, false);
      }
      if (vision.joystickHeld()) {
       
        swerve.drive(new Translation2d(-1.5*xbox.getRawAxis(1), -1.5*xbox.getRawAxis(0)), -1.5*xbox.getRawAxis(4), false, false);
      }
    }

    @Override
    public void end(boolean interrupted) {

        swerve.drive(new Translation2d(0, 0), 0, false, false);

        swerve.stopMotors();
    }

    // @Override
    // public boolean isFinished() {
    //     if (vision.horizontalAtSetpoint() && vision.rotationalAtSetpoint()) {

    //         return true;
    //     }
    //     return false;
    // }

    @Override
    public boolean isFinished() {
        if(pid.atSetpoint() && vision.rotationalAtSetpoint()) {
            return true;
        }
        return false;
    }
    
}
