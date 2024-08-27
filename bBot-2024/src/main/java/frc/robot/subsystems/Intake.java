package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class Intake extends SubsystemBase {
    private CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kIntake, MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_motor.getEncoder();
    private SparkPIDController m_PID = m_motor.getPIDController();
    
    private boolean m_enabled = false;
    private double m_velo = 5;

    public Intake() {
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setSmartCurrentLimit(0);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.burnFlash();

        m_PID.setP(0.01);
        m_PID.setI(0.0);
        m_PID.setD(0.0);
        m_PID.setFF(1.0/5676.0);
    }

    public void enable(){
        m_enabled = true;
    }

    public Command enableCMD(){
        return new InstantCommand(()-> enable());
    }

    public void disable(){
        m_enabled = false;
    }
    
    public Command disableCMD(){
        return new InstantCommand(()-> disable());
    }
    
    public void setVelo(double velo){
        m_velo = velo;
    }
    public Command setVeloCMD(double velo){
        return new InstantCommand(()-> setVelo(velo));
    }
    
    public double getTargetVelo(){
        return m_velo;
    }

    public double getActualVelo(){
        return m_encoder.getVelocity();
    }
    
    @Override
    public void periodic(){
        if(m_enabled) {
            m_PID.setReference(m_velo, ControlType.kVelocity);
        } else {
            m_motor.stopMotor();
        }
        
        SmartDashboard.putBoolean("Intake Enabled", m_enabled);
        SmartDashboard.putNumber("Intake Target Velo", m_velo);
        SmartDashboard.putNumber("Intake Actual Velo", getActualVelo());
        
    }
}