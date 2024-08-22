package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeMotor = new CANSparkMax(CanIDConstants.kIntakeMotor1, MotorType.kBrushless);
    private RelativeEncoder m_intakeEncoder = m_intakeMotor.getEncoder();
    private SparkPIDController m_intakePID = m_intakeMotor.getPIDController();
    
    private CANSparkMax m_intakeMotor2 = new CANSparkMax(CanIDConstants.kIntakeMotor2, MotorType.kBrushless);
    private RelativeEncoder m_intakeEncoder2 = m_intakeMotor2.getEncoder();
    private SparkPIDController m_intakePID2 = m_intakeMotor2.getPIDController();
    
    private boolean m_enabled = false;
    private double m_velo = 5;

    public Intake() {
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setInverted(false);
        m_intakeMotor.setSmartCurrentLimit(0);
        m_intakeMotor.enableVoltageCompensation(12.0);
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        m_intakeMotor.burnFlash();

        m_intakePID.setFeedbackDevice(m_intakeEncoder);
        m_intakePID.setP(0.01);
        m_intakePID.setI(0.0);
        m_intakePID.setD(0.0);
        m_intakePID.setFF(1.0/5676.0);
        
        m_intakeMotor2.restoreFactoryDefaults();
        m_intakeMotor2.setInverted(true);
        m_intakeMotor2.setSmartCurrentLimit(20);
        m_intakeMotor2.enableVoltageCompensation(12.0);
        m_intakeMotor2.setIdleMode(IdleMode.kBrake);
        m_intakeMotor2.burnFlash();
        
        m_intakePID2.setFeedbackDevice(m_intakeEncoder2);
        m_intakePID2.setP(0.01);
        m_intakePID2.setI(0.0);
        m_intakePID2.setD(0.0);
        m_intakePID2.setFF(1.0/5676.0);
    }


}