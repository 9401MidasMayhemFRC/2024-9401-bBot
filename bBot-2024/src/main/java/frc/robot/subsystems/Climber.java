package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class Climber extends SubsystemBase {

    private Solenoid m_solenoid = new Solenoid(CanIDConstants.kPneumatic,PneumaticsModuleType.CTREPCM, 0);
    private Compressor m_compressor = new Compressor(CanIDConstants.kPneumatic, PneumaticsModuleType.CTREPCM);

    public Climber(){
        m_solenoid.set(false);
        m_compressor.enableDigital();
    }

    public void Open(){
        m_solenoid.set(true);
    }
    
    public Command openCMD(){
        return new InstantCommand(()->Open());
    }

    public void Close(){
        m_solenoid.set(false);
    }
    
    public Command closeCMD(){
        return new InstantCommand(()-> Close());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Solenoid Open", m_solenoid.get());
    }


}
