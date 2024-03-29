// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagTrackingConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.NoteTracking.NoteTrackingLimelight;
import frc.robot.subsystems.NoteTracking.NoteTrackingPhotovision;
import frc.robot.subsystems.apriltagTracking.TagTrackingLimelight;

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

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final AHRS navxmxp;
  private final Pigeon2 pigeon2;
  private final ADXRS450_Gyro greengyro;

  private PIDController facingNotePID;
  private PIDController facingTagPID;
  private PIDController followingTagPID_X;
  private PIDController followingTagPID_Y;
  private PIDController followingTagPID_R;
  private PIDController followingTagPID;

  public PIDController faceToSpecificAnglePID;
  public PIDController moveToSpecificPointPID;
  public PIDController autosetRotation;
  // face method value maybe correct
  private final double kP = 0.08;
  private final double kI = 0;
  private final double kD = 0;

  // fix position
  public static final double kfP = 0.3;
  public static final double kfI = 0;
  public static final double kfD = 0.006; // fix position
  public static final double kPP = 0.03;
  public static final double kII = 0;
  public static final double kDD = 0;

  private double trackTargetError = 0.0;

  private Boolean trackingCondition = false;

  private NoteTrackingPhotovision note;
  private NoteTrackingLimelight Note;
  private TagTrackingLimelight tag;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  public Drivebase() {
    frontLeftLocation = new Translation2d(0.3, 0.3);
    frontRightLocation = new Translation2d(0.3, -0.3);
    backLeftLocation = new Translation2d(-0.3, 0.3);
    backRightLocation = new Translation2d(-0.3, -0.3);

    frontLeft = new SwerveModule(DrivebaseConstants.kFrontLeftDriveMotorChannel,
        DrivebaseConstants.kFrontLeftTurningMotorChannel, DrivebaseConstants.kFrontLeftTurningEncoderChannel,
        DrivebaseConstants.kFrontLeftDriveMotorInverted, DrivebaseConstants.kFrontLeftCanCoderMagOffset);
    frontRight = new SwerveModule(DrivebaseConstants.kFrontRightDriveMotorChannel,
        DrivebaseConstants.kFrontRightTurningMotorChannel, DrivebaseConstants.kFrontRightTurningEncoderChannel,
        DrivebaseConstants.kFrontRightDriveMotorInverted, DrivebaseConstants.kFrontRightCanCoderMagOffset);
    backLeft = new SwerveModule(DrivebaseConstants.kBackLeftDriveMotorChannel,
        DrivebaseConstants.kBackLeftTurningMotorChannel, DrivebaseConstants.kBackLeftTurningEncoderChannel,
        DrivebaseConstants.kBackLeftDriveMotorInverted, DrivebaseConstants.kBackLeftCanCoderMagOffset);
    backRight = new SwerveModule(DrivebaseConstants.kBackRightDriveMotorChannel,
        DrivebaseConstants.kBackRightTurningMotorChannel, DrivebaseConstants.kBackRightTurningEncoderChannel,
        DrivebaseConstants.kBackRightDriveMotorInverted, DrivebaseConstants.kBackRightCanCoderMagOffset);

    tag = new TagTrackingLimelight();
    Note = new NoteTrackingLimelight();

    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);

    navxmxp = new AHRS(Port.kMXP);
    greengyro = new ADXRS450_Gyro();
    pigeon2 = new Pigeon2(30);

    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // create the odometry
    odometry = new SwerveDriveOdometry(
        kinematics,
        gyroRotation2d(),
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

    facingNotePID = new PIDController(kP, kI, kD);
    followingTagPID = new PIDController(kfP, kfI, kfD);
    followingTagPID_X = new PIDController(
        AprilTagTrackingConstants.kPfollowX,
        AprilTagTrackingConstants.kIfollowX,
        AprilTagTrackingConstants.kDfollowX);
    followingTagPID_Y = new PIDController(
        AprilTagTrackingConstants.kPfollowY,
        AprilTagTrackingConstants.kIfollowY,
        AprilTagTrackingConstants.kDfollowY);
    followingTagPID_R = new PIDController(
        AprilTagTrackingConstants.kPfollowR,
        AprilTagTrackingConstants.kIfollowR,
        AprilTagTrackingConstants.kDfollowR);
    facingTagPID = new PIDController(kP, kI, kD);
    moveToSpecificPointPID = new PIDController(
     AprilTagTrackingConstants.kPmoveToSpecificPoint,
     AprilTagTrackingConstants.kImoveToSpecificPoint, 
     AprilTagTrackingConstants.kDmoveToSpecificPoint);
     autosetRotation = new PIDController(
      ModuleConstants.kPRotController,
      ModuleConstants.kIRotController,
      ModuleConstants.kDRotController);
  }

  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  public void setGyroReset() {
    pigeon2.reset();
    navxmxp.reset();
    greengyro.reset();

  }

  public Rotation2d gyroRotation2d() {
    if(Math.abs(pigeon2.getAngle()-navxmxp.getAngle())<2){
      return Rotation2d.fromDegrees(DrivebaseConstants.kGyroOffSet
        + ((DrivebaseConstants.kGyroInverted) ? (360.0 - pigeon2.getRotation2d().getDegrees())
            : pigeon2.getRotation2d().getDegrees()));
    }else if(Math.abs(navxmxp.getAngle()-greengyro.getAngle())!=0){
      return Rotation2d.fromDegrees(DrivebaseConstants.kGyroOffSet
        + ((DrivebaseConstants.kGyroInverted) ? (360.0 - navxmxp.getRotation2d().getDegrees())
            : navxmxp.getRotation2d().getDegrees()));
    }else{
      return Rotation2d.fromDegrees(DrivebaseConstants.kGyroOffSet
        + ((DrivebaseConstants.kGyroInverted) ? (360.0 - greengyro.getRotation2d().getDegrees())
            : greengyro.getRotation2d().getDegrees()));
    }
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
      rot = faceTargetMethod2();
    }
    swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyroRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseConstants.kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public double facingNoteRot(double currentRot) {
    var target = note.getNotes();
    if (target.size() > 0) {
      var pose = target.get(0);
      double rot = -facingNotePID.calculate(pose.getX(), 0);
      return rot;
    } else {
      return currentRot;
    }
  }

  public double newFacingNoteRot(double currentRot) {
    if (Note.getTv() == 1) {
      // var pose = target.get(0);
      double rot = -facingNotePID.calculate(Note.getTx(), 0);
      return rot;
    } else {
      return currentRot;
    }
  }

  public double[] followingNoteSpeed() {
    var target = note.getNotes();
    double[] speed = new double[3];
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    if (target.size() > 0) {
      var pose = target.get(0);
      double xSpeed = facingNotePID.calculate(pose.getY(), 0.2);
      double ySpeed = 0;
      double rot = -facingNotePID.calculate(pose.getX(), 0);
      // return rot;
      speed[0] = xSpeed;
      speed[1] = ySpeed;
      speed[2] = rot;
    }
    return speed;
  }

  public double[] newFollowingNoteSpeed() {
    // var target = note.getNotes();
    double[] speed = new double[3];
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    if (Note.getTv() == 1) {
      double xSpeed = facingNotePID.calculate(Note.getXDistance(VisionConstants.camHeight), 0.2);
      double ySpeed = 0;
      double rot = -facingNotePID.calculate(Note.getTx(), 0);
      // return rot;
      speed[0] = xSpeed;
      speed[1] = ySpeed;
      speed[2] = rot;
    }
    return speed;
  }

  public void faceTarget() {
    double offset = tag.getTx();
    double hasTarget = tag.getTv();
    double rot = 0;
    if (hasTarget == 1) {
      rot = facingTagPID.calculate(offset, 0);
    }
    drive(0, 0, -rot, false);
  }

  public double faceTargetMethod2() {
    double offset = tag.getTx();
    double hasTarget = tag.getTv();
    double rot = 0;
    if (hasTarget == 1) {
      rot = -facingTagPID.calculate(offset, 0);
    }
    SmartDashboard.putNumber("rot", rot);
    return rot;
  }

  public void setRedSpeakerPipeline() {
    tag.setCamMode(1);
    tag.setLedMode(1);
    tag.setPipeline(4);
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
        gyroRotation2d(),
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
    SmartDashboard.putNumber("gyro_heading", gyroRotation2d().getDegrees() % 360.0);
    SmartDashboard.putBoolean("trackingCondition", trackingCondition);
    tag.putDashboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    publisher.set(swerveModuleStates);
    updateOdometry();
    putDashboard();
  }
  

  /**
   * Move to specific point
   * 
   * @param Zsetpoint back and forth
   * @param Xsetpoint left and right
   */
  public double[] moveToSpecificPoint(double Zsetpoint, double Xsetpoint) {
    double Zlength = tag.getBT()[2];
    double Xlength = tag.getBT()[0];
    double Zspeed = 0;
    double Xspeed = 0;
    Zspeed = moveToSpecificPointPID.calculate(Zlength,Zsetpoint);
    Xspeed = moveToSpecificPointPID.calculate(Xlength,Xsetpoint);
    double[] speed = {Zspeed,Xspeed};
    return speed;
  }
  
    public double autosetRotation(double setPoint){
      double currentRotation = gyroRotation2d().getDegrees();
      autosetRotation.setSetpoint(setPoint);
      double rot = autosetRotation.calculate(currentRotation);
      return rot;

    } 
}
