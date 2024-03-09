package frc.robot.subsystems.apriltagTracking;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision2 extends SubsystemBase {
  private final TagTrackingPhotonvision cam1;
  private final Cam2 cam2;

  double cam1X;
  double cam2X;
  double cam1Y;
  double cam2Y;
  double x;
  double y;

  public Vision2(TagTrackingPhotonvision cam1, Cam2 cam2) {
    this.cam1 = new TagTrackingPhotonvision();
    this.cam2 = new Cam2();
  }

  public Pose2d getLatestEstimatedRobotPose() {
    cam1X = cam1.getLatestEstimatedRobotPose().getX();
    cam2X = cam2.getLatestEstimatedRobotPose().getX();
    cam1Y = cam1.getLatestEstimatedRobotPose().getY();
    cam2Y = cam2.getLatestEstimatedRobotPose().getY();

    x = (cam1X + cam2X) / 2;
    y = (cam1Y + cam2Y) / 2;

    return new Pose2d();
  }

  public void putDashboard() {
    SmartDashboard.putNumber("cam1X", cam1X);
    SmartDashboard.putNumber("cam2X", cam2X);
    SmartDashboard.putNumber("cam1Y", cam1Y);
    SmartDashboard.putNumber("cam2Y", cam2Y);

    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);
  }

}
