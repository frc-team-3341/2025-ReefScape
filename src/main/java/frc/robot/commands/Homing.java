package frc.robot.commands;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;


public class Homing extends Command{
    
    Elevator elevator;
    public Homing (Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        this.elevator.getMotorE().set(0.2);
    }
    
   @Override
   public void end(boolean interrupted) {
        this.elevator.resetEncoder();
   }

   @Override
    public boolean isFinished() {
        return this.elevator.isFWDPressed();
    }
}
