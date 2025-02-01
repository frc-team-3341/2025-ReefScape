package frc.robot.subsystems.swerve.targeting;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase{

    PhotonCamera camera;
    PhotonPoseEstimator photonEstimator;

    //Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    
    Transform3d targetData;
    double[] array = {-0.03, 0.03};
    XboxController cont = new XboxController(0);

    public Vision() {
        this.camera = new PhotonCamera("CamGoneWrong");
        this.photonEstimator = new PhotonPoseEstimator(Constants.Vision.TagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, Constants.Vision.RobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        camera.setPipelineIndex(0);

         // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(Constants.Vision.TagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, Constants.Vision.RobotToCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

    //returns whether a target (AprilTag) has been detected
    public boolean targetDetected() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            return true;
        }
        return false;
    }

    // public double getYaw() {
    //     if (targetDetected()) {
    //         PhotonPipelineResult result = camera.getLatestResult();

    //         PhotonTrackedTarget target = result.getBestTarget();
        
    //         if (target != null) {
    //             double yaw = target.getYaw();
            
    //             return yaw;
    //         }
    //     }
    //     return 0.0;
        
    // }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            photonEstimator.setReferencePose(prevEstimatedRobotPose);
            return photonEstimator.update(camera.getLatestResult());
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
            if (getHorizontalDisplacement() < array[0]) {
                direction = -1;
            }
            else if (getHorizontalDisplacement() > array[1]) {
                direction = 1;
            }
            else direction = 0;

            return direction;
        }
        return 0.0;
    }

    // public Pose3d getPose() {
    //     field.loadAprilTagLayoutField();
    //     fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    //    photonEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, getTargetData());
    //     returnphotonEstimator.getReferencePose();
        
    // }

    public boolean rotationalAtSetpoint() {
        if (getZAngle() < 178 || getZAngle() > 182) {
            return false;
        }
        return true;
    }

    public void switchHorizontalSetpoint() {
        if(cont.getLeftBumperButtonPressed()) {
            array[0] = -0.35;
            array[1] = -0.3;
        }
        else if(cont.getRightBumperButtonPressed()) {
            array[0] = 0.3;
            array[1] = 0.35;
        }
        else if(cont.getBButtonPressed()) {
            array[0] = -0.03;
            array[1] = 0.03;
        }
    }


    public boolean horizontalAtSetpoint() {
        if (getHorizontalDisplacement() < array[0] || getHorizontalDisplacement() > array[1]) {
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
        // SmartDashboard.putNumber("X pose", getPose().getX());
    }

}
