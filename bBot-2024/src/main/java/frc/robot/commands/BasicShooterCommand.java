package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/*** Make a basic command that takes in a setpoint for RackPinion as well as Velocity for the Shooter ***/

public class BasicShooterCommand extends Command{

    private boolean m_finished = false;
    
    BasicShooterCommand(){


        addRequirements();
    }

    // When the command starts what should it do?
    @Override
    public void initialize() {
        
    }

    /*While the command is running what should it check for/change while doing the command?
      Make sure to set m_finished to true when you finish the command in here
    */
    @Override
    public void execute() {
       
    }

    //What should the robot when the command finishes?
    @Override
    public void end(boolean interrupted) {
        
    }

    //Runs to see if m_finished is true to see if it is time to end the command
    @Override
    public boolean isFinished() {
        return m_finished;
    }

}
