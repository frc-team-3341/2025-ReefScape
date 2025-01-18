package frc.robot.commands.swerve.targeting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.swerve.targeting.Vision;

public class Alignment extends Command{
    SwerveDriveTrain swerve;
    Vision vision;

    double rotDirection;
    double horizDirection;

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
      if (vision.targetDetected() && (!vision.rotationalAtSetpoint() || !vision.horizontalAtSetpoint())) {
        rotDirection = vision.getRotationalDirection();
        horizDirection = vision.getHorizontalDirection();

        swerve.drive(new Translation2d(0, 0.3*horizDirection), 0.3*rotDirection, false, false);
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

            swerve.drive(new Translation2d(0, 0), 0, false, false);

            swerve.stopMotors(); 
            
            return true;
        }
        return false;
    }
    
}
