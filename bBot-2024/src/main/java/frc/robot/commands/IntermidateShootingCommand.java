package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

/*** A command that takes a distance from the tag based off of the alliance that rotates and adjusts to the distance ***/

public class IntermidateShootingCommand extends Command {

    private final PIDController m_pid = new PIDController(0.125, 0.010, 0.0);
    private Supplier<Pose2d> getPose;
    private double desiredRot;

    private CommandXboxController m_controller;

    private Drivetrain m_drive;
    private Shooter m_shooter;
    private RackPinion m_rack;

    private InterpolatingDoubleTreeMap m_angleTable = MathUtils.pointsToTreeMap(ShootingConstants.kAngleTable);
    private InterpolatingDoubleTreeMap m_veloTable = MathUtils.pointsToTreeMap(ShootingConstants.kVeloTable);

    private PhotonCamera m_cam;

    private boolean m_finished = false;

    private double manualHoodValue = 5.0;
    private boolean manualHoodOverride = false;
    private double manualVelocityValue = 70.0;
    private boolean manualVelocityOverride = false;

    public IntermidateShootingCommand(Drivetrain drivetrain, Shooter shooter, RackPinion rackPinion, Supplier<Pose2d> getPose, PhotonCamera cam, CommandXboxController controller){

        m_drive = drivetrain;
        shooter = m_shooter;
        rackPinion = m_rack;

        addRequirements(m_drive, shooter, m_rack);

        this.getPose = getPose;

        m_cam = cam;
        m_controller = controller;

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

        manualHoodOverride = SmartDashboard.getBoolean("Manual Hood Override", false);
        manualVelocityOverride = SmartDashboard.getBoolean("Manual Velocity Override", false);

        double xInput = -m_controller.getLeftY();
        double yInput = -m_controller.getLeftX();

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            xInput = -xInput;
            yInput = -yInput;
        }

        double desiredTrans[] = MathUtils.inputTransform(xInput, yInput);

        double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond*0.4;

        desiredTrans[0] *= maxLinear;
        desiredTrans[1] *= maxLinear;

        m_drive.drive(desiredTrans[0], desiredTrans[1], (desiredRot), true, true);

        if (manualHoodOverride && manualVelocityOverride) {
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            m_rack.setPose(manualHoodValue);
            m_shooter.setVelo(manualVelocityValue);
        } else if (manualHoodOverride) {
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            m_rack.setPose(manualHoodValue);
        } else if (manualVelocityOverride) {
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            m_shooter.setVelo(manualVelocityValue);
        } else {
            if (m_cam.getLatestResult().hasTargets() && alliance.isPresent()){

                if (alliance.get() == DriverStation.Alliance.Red){

                    if(m_cam.getLatestResult().getBestTarget().getFiducialId() == 4){

                        double cameraHeight = VisionConstants.robotToCam.getZ();
                        double cameraPitch = VisionConstants.robotToCam.getRotation().getY();
                        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, 57.13, cameraPitch, m_cam.getLatestResult().getBestTarget().getPitch()) * 39.37;

                        double angleSetpoint = m_angleTable.get(distance);
                        double veloSetpoint = m_veloTable.get(distance);

                        m_rack.setPose(angleSetpoint);
                        m_shooter.setVelo(veloSetpoint);

                        // assuming that cam returns in radians
                        double pidAngle = m_cam.getLatestResult().getBestTarget().getYaw() - getPose.get().getRotation().getRadians();
                        // assuming that cam returns in degrees
                        // double pidAngle = (m_cam.getLatestResult().getBestTarget().getYaw() * (Math.PI/180)) - getPose.get().getRotation().getRadians();
                        desiredRot = m_pid.calculate(pidAngle);

                    } else {
                        m_finished = true;
                    }

                } else if (alliance.get() == DriverStation.Alliance.Blue){

                    if(m_cam.getLatestResult().getBestTarget().getFiducialId() == 7){

                        double cameraHeight = VisionConstants.robotToCam.getZ();
                        double cameraPitch = VisionConstants.robotToCam.getRotation().getY();
                        double distance = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, 57.13, cameraPitch, m_cam.getLatestResult().getBestTarget().getPitch()) * 39.37;

                        double angleSetpoint = m_angleTable.get(distance);
                        double veloSetpoint = m_veloTable.get(distance);

                        m_rack.setPose(angleSetpoint);
                        m_shooter.setVelo(veloSetpoint);

                        // assuming that cam returns in radians
                        double pidAngle = m_cam.getLatestResult().getBestTarget().getYaw() - getPose.get().getRotation().getRadians();
                        // assuming that cam returns in degrees
                        // double pidAngle = (m_cam.getLatestResult().getBestTarget().getYaw() * (Math.PI/180)) - getPose.get().getRotation().getRadians();
                        desiredRot = m_pid.calculate(pidAngle);

                    } else {
                        m_finished = true;
                    }

                } else {
                    m_finished = true;
                }

            }

        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_rack.setPose(5.0);
        m_shooter.setVelo(0.0);

        
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
    
}
