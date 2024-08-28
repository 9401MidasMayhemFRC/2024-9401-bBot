package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class IntakeNote extends Command {

    private Wrist m_wrist;
    private Intake m_intake;
    private Feeder m_feeder;
    private Indexer m_indexer;

    private boolean m_finished = false;
    private Timer m_timer = new Timer();
    private boolean m_timerEnabled = false;
    
    public IntakeNote(Wrist wrist, Intake intake, Feeder feeder, Indexer indexer) {

        m_wrist = wrist;
        m_intake = intake;
        m_feeder = feeder;
        m_indexer = indexer;

        addRequirements(m_wrist, m_intake, m_feeder, m_indexer);

    }

    // Method called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_finished = false;
        m_timerEnabled = false;
        m_timer.stop();
        m_timer.reset();
        m_wrist.setPosition(5.0);
        m_feeder.setSpeedPercent(55.5);
        m_indexer.setSpeedPercent(55.5);
        m_intake.setVelo(5676.0/2);
    }

    // Method called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if((m_intake.getCurrent() >= 22.0) && !m_timerEnabled){
            m_timerEnabled = true;
            m_timer.start();
        }

        if (m_timerEnabled && (m_timer.get() >= 2.5)){
            m_wrist.setPosition(0.0);
            m_intake.disable();
            m_timer.stop();
        }
    
        if (m_timerEnabled && (m_feeder.getCurrent() > 17.0)){
            m_finished = true;
        }
        
    }

    // Method called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(interrupted){
            m_wrist.setPosition(0.0);
            m_intake.disable();
            m_timer.stop();
        }

        m_indexer.stopIndexer();
        m_feeder.stopFeeder();
        
        //if needed kickback with timer in lieu of feeder current detection

    }

    // Method that when call returns wither this command has been completed or not.
    @Override
    public boolean isFinished() {
        return m_finished;
    }

}

