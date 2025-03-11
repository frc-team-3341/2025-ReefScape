package frc.robot.subsystems.targeting;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{

    private final PhotonCamera bottomCamera = new PhotonCamera(Constants.VisionConstants.kBottomCameraName);

    Transform3d targetData;

    //private final PhotonCamera rightCamera;
    private final PhotonPoseEstimator bottomPhotonPoseEstimator;
    //private final PhotonPoseEstimator rightphotonEstimator;
    private Matrix<N3, N1> curStdDevs;
    
    double[] array = {-0.03, 0.03};
    XboxController cont = new XboxController(0);
    Joystick joy = new Joystick(0);

    ArrayList<Double> horizVals = new ArrayList<Double>();
    ArrayList<Double> rotVals = new ArrayList<Double>();

    double pidVal;

    boolean enablePoseEst = true;

    public Vision() {
        bottomCamera.setPipelineIndex(0);
        //rightCamera = new PhotonCamera(VisionConstants.kRightCameraName);

        bottomPhotonPoseEstimator =
                new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kBottomRobotToCam);
        bottomPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
    }

    public boolean targetDetected() {
        return targetData != null;
    }

    //returns whether a target (AprilTag) has been detected
    /** 
    public boolean targetDetected() {
        List<PhotonPipelineResult> results = bottomCamera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            if (result.hasTargets()) {
                return true;
            }
        }
        return false;
    }
        */

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

    //gets target data such as x and y offset, rotational offset, and returns everything as a Transform3d 
    public Transform3d getTargetData() {
        //List<PhotonPipelineResult> results = bottomCamera.getAllUnreadResults();

        //for (PhotonPipelineResult result : results) {
            PhotonPipelineResult result = bottomCamera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                if (target != null) {
                    return target.getBestCameraToTarget();
                }
            }
        //}   
        return null;
        }

    //returns the current horizontal displacement with respect to the AprilTag (uses getY() because the Y offset in PhotonVision is the horizontal axis)
    public double getHorizontalDisplacement() {
        if (targetData != null) {
            return targetData.getY();
        }
        return 0;
    }
    
    public double getLongitudinalDisplacement() {
        if (targetData != null) {
            return targetData.getX();
        }    
        return 0;
    }
    
    public double getZAngle() {
        if (targetData != null) {
            Rotation3d rot = targetData.getRotation();
            return Math.toDegrees(rot.getAngle());
        }
        return 0;
    }

    public double getRotationalDirection() {
        double direction = 0;
        if (targetData != null) {
            if (getZAngle() > 182) { //counterclockwise turn
                direction = -1;
            }
            else if (getZAngle() < 178) { //clockwise turn 
                direction = 1;
            }
        }
        return direction;
    }


    public double getHorizontalDirection() {
        double direction = 0;
        if (targetData != null) {
            if (getHorizontalDisplacement() < array[0]) {
                direction = -1;
            }
            else if (getHorizontalDisplacement() > array[1]) {
                direction = 1;
            }
        }
        return direction;
    }

    // public Pose3d getPose() {
    //     field.loadAprilTagLayoutField();
    //     fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    //     poseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, getTargetData());
    //     return poseEstimator.getReferencePose();
        
    // }

   

    public boolean rotationalAtSetpoint() {
        if (getZAngle() < 178 || getZAngle() > 182) {
            return false;
        }
        return true;
    }

    // public void switchHorizontalSetpoint() {
    //     if(cont.getLeftBumperButtonPressed()) {
    //         array[0] = -0.35;
    //         array[1] = -0.27;
    //     }
    //     else if(cont.getRightBumperButtonPressed()) {
    //         array[0] = 0.3;
    //         array[1] = 0.35;
    //     }
    //     else if(cont.getBButtonPressed()) {
    //         array[0] = -0.03;
    //         array[1] = 0.03;
    //     }
    // }

    public void switchHorizontalSetpoint() {
        if(cont.getLeftBumperButtonPressed()) {
            pidVal = -0.17;
        }
        else if(cont.getRightBumperButtonPressed()) {
            pidVal = 0.16;
        }
        else if(cont.getBButtonPressed()) {
            pidVal = 0;
        }
    }

    public double getSetpoint() {
        return pidVal;
    }

    public boolean horizontalAtSetpoint() {
        if (getHorizontalDisplacement() < array[0] || getHorizontalDisplacement() > array[1]) {
            return false;
        }
        return true;
    }

    public double getLastHorizPosition() {
        return horizVals.get(0);
    }

    public double getLastRotAngle() {
        return rotVals.get(0);
    }


    public boolean joystickHeld() {
        if (Math.abs(cont.getRawAxis(0)) > 0.1 ||  
         Math.abs(cont.getRawAxis(1)) > 0.1 || 
         Math.abs(cont.getRawAxis(4)) > 0.1 ) {
            return true;
        }
        return false;
    }

    public boolean approachingSetpoint() {
        if(getZAngle() > 175 && getZAngle() < 185) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        //Only run this when not running pose est This is really janky I know
        if (!enablePoseEst) {
            //update targetData with current info
            targetData = getTargetData();
            if (targetData != null) {
                horizVals.add(0, getHorizontalDisplacement());
                rotVals.add(0, getZAngle());
            } 
        }
        

        // SmartDashboard.putNumber("axis val", cont.getRawAxis(0));
        // SmartDashboard.putNumber("axis val 2", cont.getRawAxis(1));
        // SmartDashboard.putNumber("axis val 3", cont.getRawAxis(4));
        // SmartDashboard.putNumber("axis val 4", cont.getRawAxis(5));

        //output values to SmartDashboard/Shuffleboard
        // SmartDashboard.putBoolean("Target Detected", targetDetected());
        // // SmartDashboard.putNumber("Yaw Angle", getYaw());
        // SmartDashboard.putNumber("Z Angle", getZAngle());

        // SmartDashboard.putNumber("Horizontal Displacement", getHorizontalDisplacement());
        // SmartDashboard.putNumber("Longitudinal Displacement", getHorizontalDisplacement());
        // SmartDashboard.putNumber("X pose", getPose().getX());
    }

    public void disablePoseEst() {
        enablePoseEst = false;
    }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public Optional<EstimatedRobotPose> getLeftCameraEstimatedGlobalPose() {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         for (var change : bottomCamera.getAllUnreadResults()) {
            visionEst = bottomPhotonPoseEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
         }
         return visionEst;
     }
 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     private void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = VisionConstants.kSingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = VisionConstants.kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = bottomPhotonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs = VisionConstants.kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs = estStdDevs;
             }
         }
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getLeftCameraEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }

}