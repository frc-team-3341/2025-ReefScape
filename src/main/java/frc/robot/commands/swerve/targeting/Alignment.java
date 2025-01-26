package frc.robot.commands.swerve.targeting;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.swerve.targeting.Vision;

public class Alignment extends Command{
    SwerveDriveTrain swerve;
    Vision vision;

    double rotDirection;
    double horizDirection;
    double lHorizDirection;
    double lRotDirection;

    public Alignment(SwerveDriveTrain swerve, Vision vision) {
        this.vision = vision;
        this.swerve = swerve;

        addRequirements(this.vision, this.swerve);
    }

    @Override
    public void initialize() {
        swerve.drive(new Translation2d(0,0), 0, false, false);
    }

    @Override
    public void execute() {
      vision.switchHorizontalSetpoint();
      if (vision.targetDetected() && (!vision.rotationalAtSetpoint() || !vision.horizontalAtSetpoint())) {
        rotDirection = vision.getRotationalDirection();
        horizDirection = vision.getHorizontalDirection();

        swerve.drive(new Translation2d(0, 0.5*horizDirection), 0.5*rotDirection, false, false);
      }
      else if (!vision.targetDetected() && vision.getLastHorizPosition() != 0) {
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
        
        swerve.drive(new Translation2d(0, 0.2*lHorizDirection), 0.2*lRotDirection, false, false);
      }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);

        swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if (vision.horizontalAtSetpoint() && vision.rotationalAtSetpoint()) {

            return true;
        }
        return false;
    }
    
}
