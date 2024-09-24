package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class Indexer extends SubsystemBase{
    //neo 550
    private CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kIndexer, MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_motor.getEncoder();
    private SparkPIDController m_PID = m_motor.getPIDController();

    public Indexer(){
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setSmartCurrentLimit(0);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setIdleMode(IdleMode.kBrake);
        
        m_PID.setP(0.000005);
        m_PID.setFF(1.0/11000);
        m_motor.burnFlash();
    }
    
    public void setSpeedPercent(double percent){
        m_motor.set(percent/100);
    }

    public void setSpeedDecimal(double decimal){
        m_motor.set(decimal);
    }

    public Command setSpeedPercentCMD(double percent){
        return new InstantCommand(()-> setSpeedPercent(percent));
    }

    public Command setSpeedDecimalCMD(double decimal){
        return new InstantCommand(()-> setSpeedDecimal(decimal));
    }

    public void stopIndexer(){
        m_motor.stopMotor();
    }

    public double getSpeedPercent(){
        return (m_motor.get()) * 100;
    }
    
    public double getSpeedDecimal(){
        return m_motor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer Motor Speed in Percent", getSpeedPercent());
        SmartDashboard.putNumber("Indexer Motor Speed in Decimal", getSpeedDecimal());
    }
}
