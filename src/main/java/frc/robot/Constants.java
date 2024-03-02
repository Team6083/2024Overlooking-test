// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kMainControllerPort = 0;
  }

  public final class DrivebaseConstants {
    // drive motor channel
    public static final int kFrontLeftDriveMotorChannel = 11;
    public static final int kFrontRightDriveMotorChannel = 15;
    public static final int kBackLeftDriveMotorChannel = 13;
    public static final int kBackRightDriveMotorChannel = 17;

    // turning motor channel
    public static final int kFrontLeftTurningMotorChannel = 12;
    public static final int kFrontRightTurningMotorChannel = 16;
    public static final int kBackLeftTurningMotorChannel = 14;
    public static final int kBackRightTurningMotorChannel = 18;

    // turnning encoder channel
    public static final int kFrontLeftTurningEncoderChannel = 31;
    public static final int kFrontRightTurningEncoderChannel = 32;
    public static final int kBackLeftTurningEncoderChannel = 33;
    public static final int kBackRightTurningEncoderChannel = 34;

    // // can coder magnet offset value
    public static final double kFrontLeftCanCoderMagOffset = -0.087891;
    public static final double kFrontRightCanCoderMagOffset = -0.457764;
    public static final double kBackLeftCanCoderMagOffset = 0.357178;
    public static final double kBackRightCanCoderMagOffset = -0.345703;
    // public static final double kFrontLeftCanCoderMagOffset = -0.079590;
    // public static final double kFrontRightCanCoderMagOffset = -0.458984;
    // public static final double kBackLeftCanCoderMagOffset = 0.355225;
    // public static final double kBackRightCanCoderMagOffset = -0.333984 ;

    public static final double kMaxSpeed = 5;
    public static final double kMinSpeed = 0.25;
    public static final double kMinJoyStickValue = 0.3;
    public static final double kMaxAngularSpeed = 2.5 * Math.PI; // 1/2 rotation per second

    public static final double xLimiterRateLimit = 3.0;
    public static final double yLimiterRateLimit = 3.0;
    public static final double rotLimiterRateLimit = 3.0;

    public static final boolean kFrontLeftDriveMotorInverted = true;
    public static final boolean kFrontRightDriveMotorInverted = false;
    public static final boolean kBackLeftDriveMotorInverted = true;
    public static final boolean kBackRightDriveMotorInverted = false;

    public static final boolean kGyroInverted = false; // wheather gyro is under the robot

    public static final double kGyroOffSet = 0.0;
  }

  public static final class ModuleConstants {
    public static final double kWheelRadius = 0.046;

    public static final double kWheelDiameterMeters = 0.15;

    public static final double kMaxModuleDriveVoltage = 12.0;

    public static final double kDriveClosedLoopRampRate = 0;// 1 second 1 unit
    public static final double kTurningClosedLoopRampRate = 0.25;

    public static final double kDesireSpeedtoMotorVoltage = kMaxModuleDriveVoltage / DrivebaseConstants.kMaxSpeed;

    public static final double kMaxModuleTuringVoltage = 5.0;

    public static final double kMaxSpeedTurningDegree = 180.0;

    public static final double kPRotController = kMaxModuleTuringVoltage / kMaxSpeedTurningDegree;
    public static final double kIRotController = 0.0;
    public static final double kDRotController = 0.0004;

    public static final boolean kTurningMotorInverted = true;
  }

  public static final class NoteTrackingConstants {
    public static final String cameraName = "Microsoft_LifeCam_HD-3000";
    public static final int noteTrakingPipeline = 1;
    public static final double cameraHeight = 0.36;
    public static final double cameraWeight = 0.0;
    public static final double pitchDegree = -20.0;
    public static final double yawDegree = 0;
  }

  /**
   * Constants required in AprilTagSubsystem.
   */

  public static final class AprilTagConstants {
    public static final int A_pipeline = 0;
    /**
     * limelight_offset = {x, y, angle}
     */
    public static final double[] limelight_offset = { 0.5, 0.45, 60.0 };
    public static final double klimelightLensHeightInches = 0;
    public static final double klimelightMountAngleDegrees = 0;
  }

  public static final class AprilTagTrackingConstants {

    public static final double kPfollowX = 0.8;
    public static final double kIfollowX = 0.0;
    public static final double kDfollowX = 0.0;
    public static final double kPfollowY = 0.8;
    public static final double kIfollowY = 0.0;
    public static final double kDfollowY = 0.006;
    public static final double kPfollowR = 0.3;
    public static final double kIfollowR = 0.0;
    public static final double kDfollowR = 0.0;

    public static final double[] kfollowingTagPID_X = { 0.8, 0.0, 0.0 };
    public static final double[] kfollowingTagPID_Y = { 0.8, 0, 0.006 };
    public static final double[] kfollowingTagPID_R = { 0.3, 0, 0.0 };
    public static final double kPmoveToSpecificPoint = 0.0;
    public static final double kImoveToSpecificPoint = 0.0;
    public static final double kDmoveToSpecificPoint = 0.0;
  }

  public static final class VisionConstants {
    public static final String kCameraName = "Microsoft_LifeCam_HD-3000";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Pose3d kRobotToCam = new Pose3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));
    public static final Transform3d krobottocam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

}
