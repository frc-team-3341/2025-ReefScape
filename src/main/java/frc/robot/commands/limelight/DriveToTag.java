package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.util.lib.LimelightHelpers;

public class DriveToTag extends Command {

    private static final String limeLightName = "limelight-shahzhu";

    private SwerveDriveTrain swerve;
    StructPublisher<Pose2d> limeLightPosePublisher = 
    NetworkTableInstance.getDefault().getStructTopic("limeLightPose", Pose2d.struct).publish();

    public DriveToTag(SwerveDriveTrain swerve) {
        this.swerve = swerve;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

     // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double[] cameraPose_TargetSpace = LimelightHelpers.getCameraPose_TargetSpace(limeLightName);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {  
        return false;
    }

}