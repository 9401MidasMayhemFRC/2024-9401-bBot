package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;


public class Wrist extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kWrist, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkPIDController m_PID = m_motor.getPIDController();

    private boolean m_enablePID = true;

    private double m_position = 2;
    
    public Wrist(){
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(30);
        m_motor.enableVoltageCompensation(12.0);
        //m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float)1.0 );
        //m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)60.0 );
        m_motor.setInverted(false);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_PID.setP(0.00005);
        m_PID.setFF(0.00018);
        m_motor.burnFlash();
    }

    public void setPosition(double poisition){
        m_position = poisition;
    }

    public Command setPositionCMD(double position){
        return new InstantCommand(()-> setPosition(position));
    }

    public void disable(){
        m_motor.stopMotor();
        m_enablePID = false;
    }

    public Command disableCMD(){
        return new InstantCommand(()-> disable());
    }

    public void enable(){
        m_enablePID = true;
    }

    public Command enableCMD(){
        return new InstantCommand(()-> enable());
    }

    public double getCurrent(){
        return m_motor.getOutputCurrent();
    }

    public double getActualPosition(){
        return m_encoder.getPosition();
    }

    public double getTargetPosition(){
        return m_position;
    }


    @Override
    public void periodic(){


        if(m_enablePID){
            m_PID.setReference(m_position, ControlType.kPosition);
        }
        
        SmartDashboard.putNumber("Wrist Actual Pose", getActualPosition());
        SmartDashboard.putNumber("Wrist Output Current", getCurrent());
        SmartDashboard.putNumber("Wrist Target Pose", getTargetPosition());

    }
}
