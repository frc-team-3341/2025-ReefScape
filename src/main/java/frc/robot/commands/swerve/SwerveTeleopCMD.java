package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.util.lib.AsymmetricLimiter;
import frc.util.lib.ArcadeJoystickUtil;

public class SwerveTeleopCMD extends Command {
   // Initialize empty swerveDriveTrain object
   private final SwerveDriveTrain swerveDriveTrain;
   private final Joystick joystick;
   private boolean robotCentric = false;

   private double robotSpeed = .8;

   private double xMult = 1.0;
   private double yMult = 1.0;

   private ArcadeJoystickUtil joyUtil = new ArcadeJoystickUtil();

   // Slew rate limit controls
   // Positive limit ensures smooth acceleration (1000 * dt * dControl)
   // Negative limit ensures an ability to stop (0 * dt * dControl)
   private final AsymmetricLimiter translationLimiter = new AsymmetricLimiter(5.0D, 1000.0D);
   private final AsymmetricLimiter rotationLimiter    = new AsymmetricLimiter(10.0D, 10.0D);

  /**
   * Creates a SwerveTeleop command, for controlling a Swerve bot.
   *
   * @param swerve          - the Swerve subsystem
   * @param joy             - joystick controller
   */
   public SwerveTeleopCMD(SwerveDriveTrain swerve, Joystick joy) {
      this.swerveDriveTrain = swerve;
      this.joystick = joy;
      this.addRequirements(swerve);
   }

   @Override
   public void execute() {
      //Flip all controller input since xbox forward is negative
      double xVal = -this.joystick.getRawAxis(XboxController.Axis.kLeftY.value);
      double yVal = -this.joystick.getRawAxis(XboxController.Axis.kLeftX.value);
      double rotation = -this.joystick.getRawAxis(XboxController.Axis.kRightX.value);
      double translationRightTrigger = this.joystick.getRawAxis(XboxController.Axis.kRightTrigger.value);
      //this.robotCentric = this.joystick.getRawButtonPressed(XboxController.Button.kX.value);

      double rightTriggerVal = Math.abs(translationRightTrigger);

      if (rightTriggerVal < 0.1) {
         rightTriggerVal = 0.1;
      }

      // Inverts the speed control, so that the user can slow down instead of speeding up
      if (Constants.invertSpeedControl) {
         rightTriggerVal = 1.0 - rightTriggerVal;
      }

      //Apply a deadband, if input is less than deadband then return zero else return input
      xVal = MathUtil.applyDeadband(xVal, Constants.SwerveConstants.deadBand);
      yVal = MathUtil.applyDeadband(yVal, Constants.SwerveConstants.deadBand);

      double rotationVal = MathUtil.applyDeadband(rotation, Constants.SwerveConstants.deadBand);

      // Apply rate limiting to rotation
      rotationVal = this.rotationLimiter.calculate(rotationVal);

      //Convert input from controller into polar coordinates
      double[] polarCoords = joyUtil.regularGamePadControls(xVal, yVal, Constants.SwerveConstants.maxChassisTranslationalSpeed);

      double newHypot = robotSpeed*translationLimiter.calculate(polarCoords[0]);

      // Deadband should be applied after calculation of polar coordinates
      newHypot = MathUtil.applyDeadband(newHypot, Constants.SwerveConstants.deadBand);

      //Through magic we can convert the controller input into a vector that can be applied to the robot
      double correctedX = rightTriggerVal * xMult * newHypot * Math.cos(polarCoords[1]);
      double correctedY =  rightTriggerVal * yMult * newHypot * Math.sin(polarCoords[1]);

      // Drive swerveDriveTrain with values
      this.swerveDriveTrain.drive(new Translation2d(correctedX, correctedY),
            rotationVal * Constants.SwerveConstants.maxChassisAngularVelocity,
            this.robotCentric, false);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      this.swerveDriveTrain.drive(new Translation2d(0, 0), 0, true, false);
      // PLEASE SET THIS FOR SAFETY!!!
      this.swerveDriveTrain.stopMotors();
   }
}