package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveTrain;
import frc.util.lib.ArcadeJoystickUtil;
import frc.util.lib.AsymmetricLimiter;

public class CrabDrive extends Command {
   // Initialize empty swerve object
   private final SwerveDriveTrain swerve;

   // Create suppliers as object references
   private double x;
   private double y;

   private double xMult = 1.0;
   private double yMult = 1.0;

   private final ArcadeJoystickUtil joyUtil;

   // Slew rate limit controls
   // Positive limit ensures smooth acceleration (1000 * dt * dControl)
   private final AsymmetricLimiter translationLimiter = new AsymmetricLimiter(1000.0D, 1000.0D);

   /**
    * Creates a CrabDrive command, for controlling a Swerve bot. Please only use this for testing. Drives like a "crab"
    * without dealing with angular velocity. Drives robot centric.
    * Defaults to Blue Alliance orientation on field (X+ is foward).
    * 
    * @param swerve          - the Swerve subsystem
    * @param x               - the translational/x component of velocity 
    * @param y               - the strafe/y component of velocity
    */
   public CrabDrive(SwerveDriveTrain swerve, double x, double y) {
      this.swerve = swerve;
      // Blue Alliance by default for this testing command
      this.x = y;
      this.y = x;
      xMult = -1.0;
      this.joyUtil = new ArcadeJoystickUtil();
      this.addRequirements(swerve);
   }

   @Override
   public void execute() {

      // Get values of controls and apply deadband
      double xVal = MathUtil.applyDeadband(-this.x, Constants.SwerveConstants.deadBand); // Flip for XBox support
      double yVal = MathUtil.applyDeadband(this.y, Constants.SwerveConstants.deadBand);

      double[] output = new double[2];
      if (Constants.xboxEnabled) {
         output = joyUtil.regularGamePadControls(xVal, yVal, Constants.SwerveConstants.maxChassisTranslationalSpeed);
      } else {
         // Function to map joystick output to scaled polar coordinates
         output = joyUtil.convertXYToScaledPolar(xVal, yVal, Constants.SwerveConstants.maxChassisTranslationalSpeed);
      }

      double newHypot = translationLimiter.calculate(output[0]);

      // Deadband should be applied after calculation of polar coordinates
      newHypot = MathUtil.applyDeadband(newHypot, Constants.SwerveConstants.deadBand);

      double correctedX = xMult * newHypot * Math.cos(output[1]);
      double correctedY = yMult * newHypot * Math.sin(output[1]);

      // Drive swerve with values
      this.swerve.drive(new Translation2d(correctedX, correctedY), 0.0, false);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
      this.swerve.drive(new Translation2d(0, 0), 0, true);
      
      // PLEASE SET THIS FOR SAFETY!!!
      this.swerve.stopMotors();
   }
}