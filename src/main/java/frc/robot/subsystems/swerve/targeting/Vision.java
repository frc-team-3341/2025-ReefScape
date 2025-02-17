package frc.robot.subsystems.swerve.targeting;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Vision extends SubsystemBase{

    PhotonCamera camera;
    PhotonPoseEstimator poseEstimator;

    AprilTagFields field;
    AprilTagFieldLayout fieldLayout;
    Transform3d targetData;
    

    double[] array = {-0.03, 0.03};
    XboxController cont = new XboxController(0);
    Joystick joy = new Joystick(0);

    ArrayList<Double> horizVals = new ArrayList<Double>();
    ArrayList<Double> rotVals = new ArrayList<Double>();

    double pidVal;

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
            if (targetData != null) {
                return targetData.getY();
            }
            return 0;
        }
        else return 0.0;
    }
    
    public double getLongitudinalDisplacement() {
        if (targetDetected()) {
            if (targetData != null) {
                return targetData.getX();
            }
            return 0;
        }
        else return 0;
    }
    
    public double getZAngle() {
        if (targetDetected()) {
            if (targetData != null) {
                Rotation3d rot = targetData.getRotation();
                return Math.toDegrees(rot.getAngle());
            }
            return 0;
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
    //     poseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, getTargetData());
    //     return poseEstimator.getReferencePose();
        
    // }

   

    public boolean rotationalAtSetpoint() {
        if (getZAngle() < 179 || getZAngle() > 181) {
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
            pidVal = -0.31;
        }
        else if(cont.getRightBumperButtonPressed()) {
            pidVal = 0.31;
        }
        else if(cont.getBButtonPressed()) {
            pidVal = 0;
        }
    }

    

    public double getSetpoint() {
        return pidVal;
    }




    // public boolean horizontalAtSetpoint() {
    //     if (getHorizontalDisplacement() < array[0] || getHorizontalDisplacement() > array[1]) {
    //         return false;
    //     }
    //     return true;
    // }

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

    @Override
    public void periodic() {
        //update targetData with current info
        targetData = getTargetData();
        if (targetDetected()) {
            horizVals.add(0, getHorizontalDisplacement());
            rotVals.add(0, getZAngle());
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

}
