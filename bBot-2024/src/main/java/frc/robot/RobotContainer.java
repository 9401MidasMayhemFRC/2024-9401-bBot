// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Auto;
import frc.robot.commands.DriveByController;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntermidateShootingCommand;
import frc.robot.commands.ZeroShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Drivetrain m_drive = new Drivetrain();
  private Intake m_intake = new Intake();
  private Wrist m_wrist = new Wrist();
  private Indexer m_indexer = new Indexer();
  private Climber m_climber = new Climber();
  private Feeder m_feeder = new Feeder();
  private RackPinion m_rackPinion = new RackPinion();
  private Shooter m_shooter = new Shooter();
  private PhotonCamera m_cam = new PhotonCamera("camera");

  private PoseEstimator m_poseEstimator = new PoseEstimator(m_drive, m_cam);
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final DriveByController m_driveCommand = new DriveByController(m_drive, m_driverController);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private void configureAutoEvents() {

    //NamedCommands.registerCommand("Name of Command", actual command);

  }

  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(m_poseEstimator::getPose,
        m_poseEstimator::resetOdometry,
        m_drive::getChassisSpeed,
        m_drive::drive,
        Auto.autoConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, m_drive);
  }


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive.setDefaultCommand(m_driveCommand);
    // Configure the trigger bindings
    configureAutoBuilder();
    configureBindings();
    configureAutoEvents();
    configureAutoChooser();
  }

  private void configureAutoChooser() {
    m_chooser = AutoBuilder.buildAutoChooser("Do Nothing");

    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    m_driverController.pov(180).onTrue(new InstantCommand(
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(15.20, 5.56), new Rotation2d()));
          } else {
            m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(1.34, 5.57), new Rotation2d(Math.PI)));
          }
        }));
    m_driverController.pov(0).onTrue(new InstantCommand(
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(15.20, 5.56), new Rotation2d(Math.PI)));
          } else {
            m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(1.34, 5.57), new Rotation2d(0)));
          }
        }));

    m_driverController.leftBumper().whileTrue(new IntakeNote(m_wrist, m_intake, m_feeder, m_indexer));

    m_driverController.leftTrigger().whileTrue(new IntermidateShootingCommand(m_drive, m_shooter, m_rackPinion, m_poseEstimator::getPose, m_cam, m_driverController));

    m_driverController.rightTrigger().onTrue(new InstantCommand(()-> { m_feeder.setSpeedPercent(25);
                                                                       m_indexer.setSpeedPercent(55.5);
                                                                       m_intake.setVelo(5676.0/2);}))
                                    .onFalse(new InstantCommand(()-> { m_feeder.stopFeeder();
                                                                       m_indexer.stopIndexer();
                                                                       m_intake.setVelo(0.0);}));

    m_driverController.y().onTrue(new ZeroShooter(m_rackPinion));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}