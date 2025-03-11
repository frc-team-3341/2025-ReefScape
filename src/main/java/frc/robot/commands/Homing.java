package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class Homing extends Command{
    
    Elevator elevator;
    double output;
    public Homing (Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
        this.elevator.homeElevatorDown();
    }

    @Override
    public void execute() {
        // output = -0.6*()
        this.elevator.getMotor().set(-0.6);
    }
    
   @Override
   public void end(boolean interrupted) {
        this.elevator.resetEncoder();
        //this.elevator.getMotorE().set(0);
   }

   @Override
    public boolean isFinished() {
        return this.elevator.isREVLimit();
    }
}
