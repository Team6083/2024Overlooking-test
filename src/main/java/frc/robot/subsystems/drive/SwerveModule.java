// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final CANcoder turningEncoder;

  private final RelativeEncoder driveEncoder;

  private final PIDController rotController;

  public SwerveModule(int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel, boolean driveInverted, double canCoderMagOffset) {

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    turningEncoder = new CANcoder(turningEncoderChannel);
    CANcoderConfiguration turningEncoderConfiguration = new CANcoderConfiguration();
    turningEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turningEncoderConfiguration.MagnetSensor.MagnetOffset = canCoderMagOffset;
    turningEncoder.getConfigurator().apply(turningEncoderConfiguration);

    driveEncoder = driveMotor.getEncoder();

    driveMotor.setInverted(driveInverted);

    turningMotor.setInverted(true);

    driveMotor.setSmartCurrentLimit(10, 80);
    driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 40);
    driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 150);
    driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 150);
    driveMotor.setClosedLoopRampRate(ModuleConstants.kDriveClosedLoopRampRate);

    turningMotor.setSmartCurrentLimit(20);
    turningMotor.setClosedLoopRampRate(ModuleConstants.kDriveClosedLoopRampRate);

    driveMotor.setIdleMode(IdleMode.kBrake);
    turningMotor.setIdleMode(IdleMode.kBrake);

    rotController = new PIDController(ModuleConstants.kPRotController, ModuleConstants.kIRotController,
        ModuleConstants.kDRotController);
    rotController.enableContinuousInput(-180.0, 180.0);

    configDriveMotor();
    driveMotor.burnFlash();
    turningMotor.burnFlash();
    resetAllEncoder();
    clearSticklyFault();
    stopModule();
  }

  public void configDriveMotor() {
    driveMotor.enableVoltageCompensation(ModuleConstants.kMaxModuleDriveVoltage);
  }

  public void resetAllEncoder() {
    driveEncoder.setPosition(0);
  }

  public void clearSticklyFault() {
    driveMotor.clearFaults();
    turningMotor.clearFaults();
  }

  public void stopModule() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // to get the single swerveModule speed and the turning rate
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveRate(), new Rotation2d(Math.toRadians(getRotation())));
  }

  // to get the drive distance
  public double getDriveDistance() {
    return driveEncoder.getPosition() / 6.75 * 2 * Math.PI * ModuleConstants.kWheelRadius;
  }

  // calculate the rate of the drive
  public double getDriveRate() {
    return driveEncoder.getVelocity() / 60.0 / 6.75 * 2 * Math.PI * ModuleConstants.kWheelRadius;
  }

  // to get rotation of turning motor
  public double getRotation() {
    return turningEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  // to the get the postion by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(Math.toRadians(getRotation())));
  }

  public double[] optimizeOutputVoltage(SwerveModuleState goalState, double currentTurningDegree) {
    goalState = SwerveModuleState.optimize(goalState, Rotation2d.fromDegrees(currentTurningDegree));
    double driveMotorVoltage = ModuleConstants.kDesireSpeedtoMotorVoltage * goalState.speedMetersPerSecond;
    double turningMotorVoltage = rotController.calculate(currentTurningDegree, goalState.angle.getDegrees());
    double[] moduleState = { driveMotorVoltage, turningMotorVoltage };
    return moduleState;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < DrivebaseConstants.kMinJoyStickValue) {
      stopModule();
    } else {
      var moduleState = optimizeOutputVoltage(desiredState, getRotation());
      driveMotor.setVoltage(moduleState[0]);
      turningMotor.setVoltage(moduleState[1]);
      SmartDashboard.putNumber("turningEncoder_ID" + turningEncoder.getDeviceID() + "_voltage", moduleState[0]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turningEncoder_ID" + turningEncoder.getDeviceID() + "_degree", getRotation());
  }
}
