package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

/*** Countine After GRC completely restart and comment more for documention and better understanding for others and myself***/ 

public class Amp extends Command{

    private InterpolatingDoubleTreeMap VeloTable = MathUtils.pointsToTreeMap(Constants.AmpConstants.kVeloTable);
    private InterpolatingDoubleTreeMap AngleTable = MathUtils.pointsToTreeMap(Constants.AmpConstants.kAngleTable);

    private final PIDController m_pid = new PIDController(0.125, 0.010, 0.0);
    private Supplier<Pose2d> getPose;
    private double desiredRot;

    private PhotonCamera m_cam;

    private boolean m_finished = false;

    private Drivetrain m_drive;
    private RackPinion m_rack;
    private Shooter m_shooter;
    private CommandXboxController m_controller;

    private double manualHoodValue = 5.0;
    private boolean manualHoodOverride = false;
    private double manualVelocityValue = 70.0;
    private boolean manualVelocityOverride = false;

    public Amp(Drivetrain drive,RackPinion rack,Shooter shooter,PhotonCamera cam, Supplier<Pose2d> getPose, CommandXboxController controller){

        m_drive = drive;
        m_rack = rack;
        m_shooter = shooter;
        addRequirements(m_drive,m_rack,m_shooter);

        m_controller = controller;
        m_cam = cam;
        this.getPose = getPose;

        m_pid.setIntegratorRange(-0.1,0.1);

    }

    @Override
    public void initialize() {
        m_pid.reset();
        m_finished = false;
        SmartDashboard.putBoolean("Amp Manual Velocity Override", manualVelocityOverride);
        SmartDashboard.putNumber("Amp Set Velocity Adjust", manualVelocityValue);

        SmartDashboard.putBoolean("Amp Manual Hood Override", manualHoodOverride);
        SmartDashboard.putNumber("Amp Set Hood Adjust", manualHoodValue);
    }

    @Override
    public void execute() {
        // if the cam has tags and the best target is ethier 5 or 6 run the code otherwise it ends the command
        if ((m_cam.getLatestResult().hasTargets())&&(m_cam.getLatestResult().getBestTarget().getFiducialId() == 5) || (m_cam.getLatestResult().getBestTarget().getFiducialId() == 6)){
            double cameraHeight = VisionConstants.robotToCam.getZ();
            double cameraPitch = VisionConstants.robotToCam.getRotation().getY();
            double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, 53.38, cameraPitch, m_cam.getLatestResult().getBestTarget().getPitch()) * 39.37;

            SmartDashboard.putNumber("Distance from Amp April Tag", distance);


            manualHoodOverride = SmartDashboard.getBoolean("Amp Manual Hood Override", false);
            manualVelocityOverride = SmartDashboard.getBoolean("Amp Manual Velocity Override", false);

            if (manualHoodOverride && manualVelocityOverride) {
                manualHoodValue = SmartDashboard.getNumber("Amp Set Hood Adjust", 5.0);
                manualVelocityValue = SmartDashboard.getNumber("Amp Set Velocity Adjust", 70.0);
                double angleSetpoint = manualHoodValue;
                double veloSetpoint = manualVelocityValue;
            } else if (manualHoodOverride) {
                manualHoodValue = SmartDashboard.getNumber("Amp Set Hood Adjust", 5.0);
                double angleSetpoint = manualHoodValue;
            } else if (manualVelocityOverride) {
                manualVelocityValue = SmartDashboard.getNumber("Amp Set Velocity Adjust", 70.0);
                double veloSetpoint = manualVelocityValue;
            } else {
                // if the distance is greating than 0 and is less than a setpoint of maxium distance it will set the subsystems otherwise it will end the command
                if((distance > 0.0) && (distance <= 9.0 /* just a guess for 3/4 of a foot is as far as it can go*/) ){
                    double angleSetpoint = AngleTable.get(distance);
                    double veloSetpoint = VeloTable.get(distance);
                    // assuming that cam returns in radians
                    double pidAngle = m_cam.getLatestResult().getBestTarget().getYaw() - getPose.get().getRotation().getRadians();
                    // assuming that cam returns in degrees
                    // double pidAngle = (m_cam.getLatestResult().getBestTarget().getYaw() * (Math.PI/180)) - getPose.get().getRotation().getRadians();
                    desiredRot = m_pid.calculate(pidAngle);

                } else {
                     m_finished = true;
                }
            }

            
        } else {
            m_finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
       return m_finished;
    }
}
