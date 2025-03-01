package frc.robot.commands.targeting;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.SwerveTeleopCMD;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.targeting.Vision;

public class Alignment extends Command{
    XboxController xbox = new XboxController(0);
    SwerveDriveTrain swerve;
    Vision vision;

    double rotDirection;
    double horizDirection;
    double lHorizDirection;
    double lRotDirection;

    PIDController pid = new PIDController(0.5, 0, 0.03);

    // PIDController pid2 = new PIDController(0.5, 0, 0.02);

    public Alignment(SwerveDriveTrain swerve, Vision vision) {
        this.vision = vision;
        this.swerve = swerve;

        addRequirements(this.vision, this.swerve);
        pid.setTolerance(0.02);
    }

    @Override
    public void initialize() {
        swerve.drive(new Translation2d(0,0), 0, false, false);
    }


    @Override
    public void execute() {
      
      //switch setpoint based on driver input
      vision.switchHorizontalSetpoint();
      
      //if  a target is in view, and the driver is not driving, run autonomous alignment
      if (vision.targetDetected() && !vision.joystickHeld()) { 
        rotDirection = vision.getRotationalDirection();

        horizDirection = -pid.calculate(vision.getHorizontalDisplacement(), vision.getSetpoint());

        double val = (5.0*horizDirection)+(Math.signum(horizDirection)*0.05);
        double error = (vision.getSetpoint() - vision.getHorizontalDisplacement());

        if (Math.abs(val) > 0.75) {
            val = Math.signum(horizDirection)*0.75;
        }
        else if (Math.abs(error) < 0.1 && vision.approachingSetpoint()) {
            val = Math.signum(horizDirection)*0.1;
        }
        //use below if testing with advantagescope
        SmartDashboard.putNumber("align val", val);
        System.out.println(val);
        SmartDashboard.putNumber("error", (vision.getSetpoint() - vision.getHorizontalDisplacement()));
        swerve.drive(new Translation2d(0, val), 0.6*rotDirection, false, false);
      }
      else if (!vision.targetDetected() && (vision.getLastHorizPosition() != 0 || vision.getLastRotAngle() != 0)) {

        if (vision.getLastHorizPosition() < 0) {
            lHorizDirection = -1;
        }

        else if (vision.getLastHorizPosition() > 0) {
            lHorizDirection = 1;
        }

        if (vision.getLastRotAngle() > 182 && vision.getLastHorizPosition() < 0) {
            lRotDirection = -1;
        }

        else if (vision.getLastRotAngle() < 178 && vision.getLastHorizPosition() > 0) {
            lRotDirection = 1;
        }
        
        swerve.drive(new Translation2d(0, 0.6*lHorizDirection), 0.6*lRotDirection, false, false);
      }
      if (vision.joystickHeld()) {
       
        swerve.drive(new Translation2d(-0.65*xbox.getRawAxis(1), -0.65*xbox.getRawAxis(0)), -0.8*xbox.getRawAxis(4), false, false);
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