// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kDriverControllerPort = 0;
  }

  public final class DrivetainConstants {
    public static final double kMaxSpeed = 5; // 7 meters per second
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

    public static final double kMinTrackingDistance = 5.0;
    public static final double kPTrackingValue = 0.1;
  }

  public static final class ModuleConstants {
    public static final double kWheelRadius = 0.046;

    public static final double kWheelDiameterMeters = 0.15;

    public static final double kLimitModuleDriveVoltage = 7.0;

    public static final double kMaxModuleDriveVoltage = 12.0;

    public static final double kClosedLoopRampRate = 0.25;// 1 second 1 unit

    public static final double kDesireSpeedtoMotorVoltage = kMaxModuleDriveVoltage / DrivetainConstants.kMaxSpeed;

    public static final double kMaxModuleTuringVoltage = 2.0;

    public static final double kMaxSpeedTurningDegree = 180.0;

    public static final double kPRotController = kMaxModuleTuringVoltage / kMaxSpeedTurningDegree;

    public static final double kDRotController = 0.0004;
  }

  public static final class AprilTagConstants {
    public static final double klimelightLensHeightInches = 0;
    public static final double klimelightMountAngleDegrees = 0;
  }

}
