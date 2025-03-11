package frc.robot.commands.autoPaths;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.AutoPath;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class S2_H1_C2 extends SequentialCommandGroup  {
  SwerveDriveTrain swerve;
  AutoPath autoPath;  
  
  public S2_H1_C2(SwerveDriveTrain swerve) throws FileVersionException, IOException, ParseException {
    this.swerve = swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // *TODO: tune the constants for shooting into the speaker from the left
      new AutoPath("S2_H1_C2", this.swerve, false)

    );
  }
}