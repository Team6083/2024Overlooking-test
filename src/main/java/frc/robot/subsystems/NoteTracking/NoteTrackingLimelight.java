// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.NoteTracking;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class NoteTrackingLimelight extends SubsystemBase {
  /** Creates a new visionTracking. */

  private final NetworkTable table;
  private double tv = 0.0;
  private double tx = 0.0;
  private double ty = 0.0;
  private double ta = 0.0;

  public NoteTrackingLimelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setCamMode(1);
    setLedMode(1);
  }

  public void startTracking() {
    setCamMode(0);
    setLedMode(0);
  }

  public void endTracking() {
    setCamMode(1);
    setLedMode(1);
  }

  public double getTv() {
    return table.getEntry("tv").getDouble(0.0);
  }

  public double getTx() {
    double tx = table.getEntry("tx").getDouble(0.0);
    if (tx == 0) {
      System.out.println("Can't find target");
    }
    return tx;
  }

  public double getTy() {
    return table.getEntry("ty").getDouble(0.0);
  }

  /**
   * Return bot to note distance
   * 
   * @param camHeight
   * @return bot to note distance
   */
  public double getXDistance(double camHeight) {
    // double tx = getTx();
    double degree = getNoteAngle(VisionConstants.cam_offset);
    double radian = Math.toRadians(degree);
    double distance = camHeight / Math.tan(radian);
    return distance;
  }

  /**
   * Return cam to note angle in vertical direction
   * 
   * @param cam_offset
   * @return angle (degree)
   */
  public double getNoteAngle(double cam_offset) {
    double degree = getTy() + cam_offset;
    return degree;
  }

  public double getYDistance(double camHeight) {
    double degree = getNoteTxAngle(VisionConstants.cam_offset);
    double radian = Math.toRadians(degree);
    double distance = camHeight / Math.tan(radian);
    return distance;
  }

  /**
   * Return cam to note angle in horizontal direction
   * 
   * @param cam_offset
   * @return angle (degree)
   */
  public double getNoteTxAngle(double cam_offset) {
    double degree = getTx() + cam_offset;
    return degree;
  }

  public double getTa() {
    return table.getEntry("ta").getDouble(0.0);
  }

  public void setCamMode(int camMode) {
    table.getEntry("camMode").setNumber(camMode);
  }

  public void setLedMode(int ledMode) {
    table.getEntry("ledMode").setNumber(ledMode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("limelight_Tx", tx);
  }

}
