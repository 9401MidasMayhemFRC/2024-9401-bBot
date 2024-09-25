package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/*** Countine After GRC ***/ 

public class PoseEstimator extends SubsystemBase{
    private static final AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private PhotonPoseEstimator m_poseEstimator;
    private PhotonCamera m_cam;

    private SwerveDrivePoseEstimator m_drivePose;
    private boolean m_validTag;
    private PhotonPipelineResult m_result;

    private Field2d m_field= new Field2d();

    private Drivetrain m_drive;

    public PoseEstimator(Drivetrain drive, PhotonCamera cam){
        m_drivePose = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
                drive.getGyro(),
                drive.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.229, 0.229, 0.229),
                VecBuilder.fill(10, 10, 10));

        m_cam = cam;

        m_poseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_cam, VisionConstants.robotToCam);

        m_drive = drive;

        
        
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        m_poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_poseEstimator.update();
    }

    public void resetOdometry(Pose2d pose) {
        m_drive.resetOdometry(pose.getRotation().times(-1.0));
        m_drivePose.resetPosition(m_drive.getGyro().times(1.0), m_drive.getModulePositions(), pose);
    }

    public Pose2d getPose() {

        return m_drivePose.getEstimatedPosition();

    }

    public Rotation2d getRotation2d(){
        return m_cam.getLatestResult().getMultiTagResult().estimatedPose.best.getRotation().toRotation2d();
    }

    private Pose2d getSingleTagPose(){
       return new Pose2d(m_cam.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d(), new Rotation2d(m_cam.getLatestResult().getBestTarget().getYaw()));
    }

    @Override
    public void periodic() {

        m_result = m_cam.getLatestResult();
        m_validTag = m_result.hasTargets();

        double time = Timer.getFPGATimestamp();

        m_drivePose.updateWithTime(time, m_drive.getGyro(), m_drive.getModulePositions());

        double latency = m_result.getLatencyMillis()/1000;
        Pose2d m_pose;

        if (m_validTag && m_cam.isConnected()){
            if (m_cam.getLatestResult().targets.size() > 1){
                m_pose = new Pose2d(m_cam.getLatestResult().getMultiTagResult().estimatedPose.best.getTranslation().toTranslation2d(), getRotation2d());
                SmartDashboard.putString("Pose", m_pose.toString());
                SmartDashboard.putString("Camera Results", m_cam.getLatestResult().getMultiTagResult().toString());
            }else{
                m_pose = getSingleTagPose();
                SmartDashboard.putString("Pose", m_pose.toString());
                SmartDashboard.putString("Camera Results", m_cam.getLatestResult().toString());
            }
            m_drivePose.addVisionMeasurement( m_pose , time-latency);
        } else {
            if (!m_cam.isConnected()){
                DriverStation.reportError("Camera Disconnected", true);
            } else {
                DriverStation.reportWarning("No Tags Read", false);
            }
        }

        m_field.setRobotPose(m_drivePose.getEstimatedPosition());
        SmartDashboard.putData(m_field);

    }
}
