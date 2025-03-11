package frc.robot.commands.targeting;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.robot.subsystems.targeting.Vision;

public class LongitudinalAlignment extends Command {
    SwerveDriveTrain swerve;
    Vision vision;
    boolean isAligned;
    double[] toleranceArray = {0.05};

    public LongitudinalAlignment(SwerveDriveTrain swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        
        addRequirements(this.swerve, this.vision);
    }

    @Override
    public void initialize() {
        swerve.driveRelative(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public void execute() { 
        if (vision.targetDetected() && (vision.getLongitudinalDisplacement() > toleranceArray[0])) {
            System.out.println(vision.getLongitudinalDisplacement());
            swerve.driveRelative(new ChassisSpeeds(0.5, 0, 0)); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveRelative(new ChassisSpeeds(0, 0, 0));

        swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if (vision.getLongitudinalDisplacement() <= toleranceArray[0]) {
            System.out.println("hi fellow sigma");
            return true;

        }
        return false;
    }
}