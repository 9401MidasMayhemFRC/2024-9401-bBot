// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CanIDConstants {
    public static final int kLeftShooterMotor = 10;
    public static final int kRightShooterMotor = 9;
    public static final int kFeedMotor = 11;
    public static final int kIntake = 12;
    public static final int kWrist = 13;
    public static final int kRackPinion = 14;
  }

  public static class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kTrackWidth = 24.75 / 39.37; // Center distance in meters between right and left
                                                                // wheels on
    // robot
    public static final double kWheelBase = 24.75 / 39.37; // Center distance in meters between front and back
                                                                 // wheels on
    // robot

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 2;

    public static final boolean kGyroReversed = false;
    public static final boolean kUseNEO = false;

    public static final int kFrontLeftDriveMotorPort = 1; // CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 3; // CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 5; // CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 7; // CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 2; // CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 4; // CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 6; // CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 8; // CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 0; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 2; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 3; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = 2.917;// Encoder Offset in Radians
    public static final double kFrontRightOffset = 3.035; // Encoder Offset in Radians
    public static final double kBackLeftOffset = 4.820; // Encoder Offset in Radians
    public static final double kBackRightOffset = 0.6675 - Math.PI; // Encoder Offset in Radians

    public static final double[] kFrontLeftTuningVals = { 0.015, 0.23, 0.15, 0 }; // {Static Gain, FeedForward,
                                                                                  // Proportional Gain, ModuleID for
                                                                                  // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.015, 0.23, 0.15, 1 }; // {Static Gain, FeedForward,
                                                                                   // Proportional Gain, ModuleID for
                                                                                   // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.015, 0.23, 0.15, 2 }; // {Static Gain, FeedForward,
                                                                                 // Proportional Gain, ModuleID for
                                                                                 // Tuning}
    public static final double[] kBackRightTuningVals = { 0.015, 0.23, 0.15, 3 }; // {Static Gain, FeedForward,
                                                                                  // Proportional Gain, ModuleID for
                                                                                  // Tuning}


    public static final double kWheelBaseRadius = 0.5
        * Math.sqrt(Math.pow(kWheelBase, 2) + Math.pow(kTrackWidth, 2));

    public static final double kMaxAcceleration = 3.0;
    // Conditions & Battery, Robot will not exceed this
    // speed in closed loop control
    public static final double kTestMaxAcceleration = 1.0;
    public static final double kTestMaxSpeedMetersPerSecond = 1.0;

    // but spinning fast is not particularly useful or driver
    // friendly
    public static final double kMaxAngularAccel = 1.5 * Math.PI; // Maximum Angular Speed desired. NOTE: Robot can
                                                                 // exceed this
    // but spinning fast is not particularly useful or driver
    // friendly

    public static final double kInnerDeadband = 0.02; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)

    public static final double[] kKeepAnglePID = { 1.00, 0, 0 }; // Defines the PID values for the keep angle PID

    private static final SwerveModuleState[] kLockedWheelsHelper = kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
    public static final SwerveModuleState[] kLockedWheels = {
        new SwerveModuleState(0.0, kLockedWheelsHelper[0].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[1].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[2].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[3].angle.rotateBy(new Rotation2d(Math.PI / 2)))
    };

    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 5550.0 / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.05;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static double kCubic = 0.95;
    public static double kLinear = 0.05;
    public static double kDeadband = 0.02;
  }
}
