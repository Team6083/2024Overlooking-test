// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivetain. */
  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final VisionTrackingLimelight track;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final AHRS gyro;

  private double trackTargetError = 0.0;

  private Boolean trackingCondition = false;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  public Drivebase() {
    frontLeftLocation = new Translation2d(0.3, 0.3);
    frontRightLocation = new Translation2d(0.3, -0.3);
    backLeftLocation = new Translation2d(-0.3, 0.3);
    backRightLocation = new Translation2d(-0.3, -0.3);

    frontLeft = new SwerveModule(10, 11, 5, DrivebaseConstants.kFrontLeftDriveMotorInverted,
        DrivebaseConstants.kFrontLeftCanCoderMagOffset);
    frontRight = new SwerveModule(15, 14, 4, DrivebaseConstants.kFrontRightDriveMotorInverted,
        DrivebaseConstants.kFrontRightCanCoderMagOffset);
    backLeft = new SwerveModule(12, 13, 2, DrivebaseConstants.kBackLeftDriveMotorInverted,
        DrivebaseConstants.kBackLeftCanCoderMagOffset);
    backRight = new SwerveModule(17, 16, 3, DrivebaseConstants.kBackRightDriveMotorInverted,
        DrivebaseConstants.kBackRightCanCoderMagOffset);

    track = new VisionTrackingLimelight();

    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);

    gyro = new AHRS(Port.kMXP);

    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // create the odometry
    odometry = new SwerveDriveOdometry(
        kinematics,
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    // reset the gyro
    setGyroReset();

    // set the swerve speed equal 0
    drive(0, 0, 0, false);
  }

  public void setGyroReset() {
    gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return (DrivebaseConstants.kGyroInverted) ? Rotation2d.fromDegrees(360.0 - gyro.getRotation2d().getDegrees())
        : gyro.getRotation2d();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * 
   *                      using the wpi function to set the speed of the swerve
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (trackingCondition) {
      rot = (track.getTx() - trackTargetError) * DrivebaseConstants.kPTrackingValue;
    }
    swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseConstants.kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void trackingDrive(double tx, double speed) {
    double tanValue = tx / 20;
    swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.5 * tanValue, 0.5, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseConstants.kMaxSpeed);
    Rotation2d angle = swerveModuleStates[0].angle;
    frontLeft.setDesiredState(new SwerveModuleState(speed, angle));
  }

  public void switchTrackCondition() {
    trackingCondition = !trackingCondition;
  }

  public void addTrackTargetError() {
    trackTargetError += 2;
  }

  public void minusTrackTargetError() {
    trackTargetError -= 2;
  }

  public void resetTrackTargetError() {
    trackTargetError = 0.0;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  public void putDashboard() {
    SmartDashboard.putNumber("frontLeft_speed", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("frontRight_speed", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("backLeft_speed", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("backRight_speed", swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("gyro_heading", getRotation2d().getDegrees()%360.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    putDashboard();
  }
}
