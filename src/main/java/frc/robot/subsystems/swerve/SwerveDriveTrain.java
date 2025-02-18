package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.util.lib.SwerveUtil;

/**
 * <p>Creates a SwerveDrive class.</p>
 * 
 * <p>In its current form, it can be simulated using the simulation integration method from the static SwerveUtil class. This simulation is less precise than real life, but much better than AutoDesk Synthesis :).</p>
 * 
 * @author Aric Volman
 */
public class SwerveDriveTrain extends SubsystemBase {
   // Create Navx
   private AHRS navx = new AHRS(NavXComType.kMXP_SPI);

   //Creates pdh
   private PowerDistribution pdh = new PowerDistribution(Constants.PDH_can_id, PowerDistribution.ModuleType.kCTRE);

   // Create object representing swerve modules
   private SwerveModuleIOSparkMax[] moduleIO;

   // Create object that represents swerve module positions (i.e. radians and meters)
   private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

   // Create kinematics object
   private SwerveDriveKinematics kinematics;

   private ChassisSpeeds chassisSpeeds;

   // Create poseEstimator object
   // This can fuse Visual and Encoder odometry with different standard deviations/priorities
   private SwerveDrivePoseEstimator poseEstimator;

   // Add field to show robot
   private Field2d field;
   private Rotation2d offsetNavx = new Rotation2d();
   private final StructArrayPublisher<SwerveModuleState> statePublisher;
   private final StructArrayPublisher<SwerveModuleState> targetStatePublisher;
   private final StructArrayPublisher<SwerveModuleState> absStatePublisher;
   private final StructPublisher<ChassisSpeeds> chassisSpeedsPublisher;

   private SwerveDriveSimulation mapleSimDrive;


   /**
    * Creates a new SwerveDrive object. Intended to work both with real modules and
    * simulation.
    * @author Aric Volman
    */
   public SwerveDriveTrain(Pose2d startingPose, SwerveModuleIOSparkMax FL, SwerveModuleIOSparkMax FR, SwerveModuleIOSparkMax BR, SwerveModuleIOSparkMax BL) {
      // Assign modules to their object
      this.moduleIO = new SwerveModuleIOSparkMax[] { FL, FR, BR, BL};

      // Iterate through module positions and assign initial values
      modulePositions = SwerveUtil.setModulePositions(moduleIO);   

      // Initialize all other objects
      this.kinematics = new SwerveDriveKinematics(Constants.SwerveConstants.moduleLocations);
      // Can set any robot pose here (x, y, theta) -> Built in Kalman Filter
      // FUTURE: Seed pose with CV
      // Auto is field-oriented
      this.poseEstimator = new SwerveDrivePoseEstimator(this.kinematics, Rotation2d.fromDegrees(getGyroYaw()), this.modulePositions, startingPose);
      this.field = new Field2d();
      
      this.chassisSpeeds =  new ChassisSpeeds(0.0, 0.0, 0.0);
      statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
      absStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates_abs", SwerveModuleState.struct).publish();
      targetStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates_target", SwerveModuleState.struct).publish();
      chassisSpeedsPublisher = NetworkTableInstance.getDefault().getStructTopic("/ChassisSpeeds", ChassisSpeeds.struct).publish();

      //Make sure to call this last since we want everything else to be configured
      if (Robot.isSimulation()) {
         createSimulationSwerve(startingPose);
      }
   }

   public void periodic() {
      // Update module positions
      modulePositions = SwerveUtil.setModulePositions(moduleIO);

      // Update odometry, field, and poseEstimator
      this.poseEstimator.update(this.getRotation(), this.modulePositions);
      this.field.setRobotPose(this.getPoseFromEstimator());

      // Update telemetry of each swerve module
      SwerveUtil.updateTelemetry(moduleIO);

      // Draw poses of robot's modules in SmartDashboard
      SwerveUtil.drawModulePoses(modulePositions, field, getPoseFromEstimator());

      // Put field on SmartDashboard
      SmartDashboard.putData("Field", this.field);
      SmartDashboard.putNumberArray("Actual States", SwerveUtil.getDoubleStates(getActualStates()));
      SmartDashboard.putNumberArray("Setpoint States", SwerveUtil.getDoubleStates(getSetpointStates()));
      SmartDashboard.putNumber("Robot Rotation", getPoseFromEstimator().getRotation().getRadians());
      SmartDashboard.putNumber("Angle", getHeading());

      targetStatePublisher.set(getSetpointStates());
      statePublisher.set(getActualStates());
      absStatePublisher.set(getCanCoderStates());
      chassisSpeedsPublisher.set(this.chassisSpeeds);
   }

   public void simulationPeriodic() {
      // Add simulation! Yes, with the Util class, it's that easy!
      // WARNING: This doesn't use the Navx, just the states of the modules
      SwerveUtil.addSwerveSimulation(moduleIO, getActualStates(), kinematics);
   }

   /**
    * Drive either field oriented, or not field oriented
    * 
    * @param translation   Vector of x-y velocity in m/s
    * @param rotation      Rotation psuedovector in rad/s
    * @param fieldRelative Whether or not the robot should drive field relative
    * @param isOpenLoop    Whether or not to control robot with closed or open loop
    *                      control
    */
   public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
  
      this.chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, this.getRotation())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

      this.chassisSpeeds = SwerveUtil.discretize(this.chassisSpeeds, -4.0);

      //Convert the robot vector into module states which is a vector for each module
      //Explanation found here https://samliu.dev/blog/a-deep-dive-into-swerve#16d4e0ca3f0280b19d85cdb8b2adac83
      SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

      // MUST USE SECOND TYPE OF METHOD
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, this.chassisSpeeds,
            Constants.SwerveConstants.maxWheelLinearVelocityMeters,
            Constants.SwerveConstants.maxChassisTranslationalSpeed,
            Constants.SwerveConstants.maxChassisAngularVelocity);

      for (int i = 0; i < swerveModuleStates.length; i++) {
         this.moduleIO[i].setDesiredState(swerveModuleStates[i]);
      }

   }

   /**
    * Drive the robot for PathPlannerLib
    */
   public void driveRelative(ChassisSpeeds speeds) {
      speeds = SwerveUtil.discretize(speeds, -4.0);

      SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(speeds);

      // MUST USE SECOND TYPE OF METHOD
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speeds,
            Constants.SwerveConstants.maxWheelLinearVelocityMeters,
            Constants.SwerveConstants.maxChassisTranslationalSpeed,
            Constants.SwerveConstants.maxChassisAngularVelocity);

      for (int i = 0; i < swerveModuleStates.length; i++) {
         this.moduleIO[i].setDesiredState(swerveModuleStates[i]);
      }
   }

   /**
    * Gets the SwerveModuleState[] for our use in code.
    */
   public SwerveModuleState[] getSetpointStates() {
      SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];
      for (int i = 0; i < states.length; i++) {
         states[i] = this.moduleIO[i].getDesiredState();
      }
      return states;
   }

   /**
    * Gets the actual SwerveModuleState[] for our use in code
    */
   public SwerveModuleState[] getActualStates() {
      SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];
      for (int i = 0; i < states.length; i++) {
         states[i] = this.moduleIO[i].getActualModuleState();
      }
      return states;
   }

   public SwerveModuleState[] getCanCoderStates() {
      SwerveModuleState[] states = new SwerveModuleState[moduleIO.length];
      for (int i = 0; i < states.length; i++) {
         states[i] = this.moduleIO[i].getCanCoderState();
      }
      return states;
   }

   /**
    * Sets the velocities and positions (drive, turn) of one module
    * @param driveVel Drive velocity (m/s)
    * @param turnPos Turn position (degrees)
    * @param index Index of module
    */
   public void setModuleSetpoints(double driveVel, double turnPos, int index) {
      // Precondition: Safety check within bounds
      if (index >= 0 && index < moduleIO.length) {
         SwerveModuleState state = new SwerveModuleState(driveVel, Rotation2d.fromDegrees(turnPos));
         moduleIO[index].setDesiredState(state);
      }
   }

   /**
    * Stops the motors of the swerve drive. Useful for stopping all sorts of Commands.
    */
   public void stopMotors() {
      for (SwerveModuleIOSparkMax module : moduleIO) {
         module.setDriveVoltage(0.0);
         module.setTurnVoltage(0.0);
      }
   }

   //Returns the current yaw value (in degrees, from -180 to 180)
   public double getGyroYaw() {
      return navx.getYaw();
   }

   /** TODO: FIX THIS
    * Get heading of Navx. Negative because Navx is CW positive.
    */
   public double getHeading() {
      return -navx.getRotation2d().plus(offsetNavx).getDegrees();
   }

   /** TODO: FIX THIS
    * Get rate of rotation of Navx. Negative because Navx is CW positive.
    */
   public double getTurnRate() {
      return -navx.getRate();
   }

   /** TODO: FIX THIS not good
    * Get Rotation2d of Navx. Positive value (CCW positive default).
    */
   public Rotation2d getRotation() {
      return navx.getRotation2d().plus(offsetNavx); 
   }

   /**
    * Get Pose2d of poseEstimator.
    */
   public Pose2d getPoseFromEstimator() {
      return poseEstimator.getEstimatedPosition();
   }

   /**
    * Reset pose of robot to pose
    */
   public void resetPose(Pose2d pose) {
      SmartDashboard.putNumber("Pose", pose.getX());
      poseEstimator.resetPosition(pose.getRotation(), modulePositions, pose);
      offsetNavx = pose.getRotation().minus(navx.getRotation2d());

      if (Constants.isSim) {
         mapleSimDrive.setSimulationWorldPose(pose);
      }
   }

   /**
    * Get chassis speeds for PathPlannerLib
    */
   public ChassisSpeeds getRobotRelativeSpeeds() {
      return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getActualStates()), getRotation());
   }

   /** Gets field */
   public Field2d getField() {
      return field;
   }

   public void setModulesPositions(double velocity, double angle){
      for(int i = 0; i < 4; i++){
         setModuleSetpoints(velocity, angle, i);
      }
   }

   public Command resetHeadingCommand() {
      return runOnce(() -> {
         navx.reset();   
      });
   }

   private void createSimulationSwerve(Pose2d startingPose) {
      DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
                                                                           .withBumperSize(
                                                                                 Meters.of(Constants.SwerveConstants.swerveModuleYdistance)
                                                                                       .plus(Inches.of(5)),
                                                                                 Meters.of(Constants.SwerveConstants.swerveModuleXdistance)
                                                                                       .plus(Inches.of(5)))
                                                                           .withRobotMass(Kilograms.of(Constants.SwerveConstants.robotMassInKg))
                                                                           .withCustomModuleTranslations(Constants.SwerveConstants.moduleLocations)
                                                                           .withGyro(COTS.ofNav2X())
                                                                           .withSwerveModule(new SwerveModuleSimulationConfig(
                                                                              //Hopefully these motors are right
                                                                                 DCMotor.getNEO(1),
                                                                                 DCMotor.getNEO(1),
                                                                                 Constants.ModuleConstants.driveGearRatio,
                                                                                 Constants.ModuleConstants.turnGearRatio,
                                                                                 //Should probably move to constants, might need to double check later
                                                                                 Volts.of(.02),
                                                                                 Volts.of(.03),
                                                                                 Inches.of(
                                                                                    Units.metersToInches(Constants.ModuleConstants.wheelDiameterMeters) /
                                                                                    2),
                                                                                 KilogramSquareMeters.of(0.02),
                                                                                 Constants.SwerveConstants.wheelGripCoefficientOfFriction)
                                                                                             );

      mapleSimDrive = new SwerveDriveSimulation(simulationConfig, startingPose);

      for (int i = 0; i < moduleIO.length; i++) {
         this.moduleIO[i].configureModuleSimulation(mapleSimDrive.getModules()[i]);
      }

      // register the drivetrain simulation
      SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);

      //Figure out imu later
      //simIMU = new SwerveIMUSimulation(mapleSimDrive.getGyroSimulation());
      //imuReadingCache = new Cache<>(simIMU::getGyroRotation3d, 5L);
   }

}