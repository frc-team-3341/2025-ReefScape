package frc.robot.subsystems.swerve.targeting;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;
    AprilTagFields field;
    AprilTagFieldLayout fieldLayout;
    Transform3d targetData;
    
    public Vision(PhotonCamera camera) {
        this.camera = camera;
        camera.setPipelineIndex(0);
        
    }

    //returns whether a target (AprilTag) has been detected
    public boolean targetDetected() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return true;
        }
        return false;
    }

    public double getYaw() {
        if (targetDetected()) {
            PhotonPipelineResult result = camera.getLatestResult();

            PhotonTrackedTarget target = result.getBestTarget();
        
            if (target != null) {
                double yaw = target.getYaw();
            
                return yaw;
            }
        }
        return 0.0;
        
    }

    //gets target data such as x and y offset, rotational offset, and returns everything as a Transform3d 
    public Transform3d getTargetData() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (targetDetected()) {
            PhotonTrackedTarget target = result.getBestTarget();
                if (target != null) {
                    return target.getBestCameraToTarget();
                }
            }
        return null;
        }

    //returns the current horizontal displacement with respect to the AprilTag (uses getY() because the Y offset in PhotonVision is the horizontal axis)
    public double getHorizontalDisplacement() {
        if (targetDetected()) {
            return targetData.getY();
        }
        else return 0;
    }
    
    public double getLongitudinalDisplacement() {
        if (targetDetected()) {
            return targetData.getX();
        }
        else return 0;
    }
    
    public double getZAngle() {
        if (targetDetected()) {
            Rotation3d rot = targetData.getRotation();
            return Math.toDegrees(rot.getAngle());
        }
        else return 0.0;
    }

    public double getRotationalDirection() {
        double direction;
        if (targetDetected()) {
            if (getZAngle() > 182) { //counterclockwise turn
                direction = -1;
            }
            else if (getZAngle() < 178) { //clockwise turn 
                direction = 1;
            }
            else direction = 0;

            return direction;
        }
        return 0.0;
    }

    public double getHorizontalDirection() {
        double direction;
        if (targetDetected()) {
            if (getHorizontalDisplacement() < -0.05) {
                direction = -1;
            }
            else if (getHorizontalDisplacement() > 0.05) {
                direction = 1;
            }
            else direction = 0;

            return direction;
        }
        return 0.0;
    }

    public Pose3d getPose() {
        field.loadAprilTagLayoutField();
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, getTargetData());
        return poseEstimator.getReferencePose();
        
    }

   

    public boolean rotationalAtSetpoint() {
        if (getZAngle() < 178 || getZAngle() > 182) {
            return false;
        }
        return true;
    }


    public boolean horizontalAtSetpoint() {
        if (getHorizontalDisplacement() < -0.05 || getHorizontalDisplacement() > 0.05) {
            return false;
        }
        return true;
    }

    @Override
    public void periodic() {
        //update targetData with current info
        targetData = getTargetData();

        //output values to SmartDashboard/Shuffleboard
        // SmartDashboard.putBoolean("Target Detected", targetDetected());
        // SmartDashboard.putNumber("Yaw Angle", getYaw());
        // SmartDashboard.putNumber("Z Angle", getZAngle());

        // SmartDashboard.putNumber("Horizontal Displacement", getHorizontalDisplacement());
        // SmartDashboard.putNumber("Longitudinal Displacement", getHorizontalDisplacement());
        SmartDashboard.putNumber("X pose", getPose().getX());
    }

}
