package frc.robot.subsystems.ApriltagTracking;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.AprilTagConstants;

public class TagTrackingLimelight extends SubsystemBase {
    public NetworkTable table;
    public NetworkTableEntry tx;// = table.getEntry("tx");// table.getEntry("tx");
    public NetworkTableEntry ty;// = table.getEntry("ty");
    public NetworkTableEntry ta;// = table.getEntry("ta");
    public NetworkTableEntry tv;
    public NetworkTableEntry tid;
    public NetworkTableEntry tl;

    public NetworkTableEntry BT;
    public NetworkTableEntry CR;
    public NetworkTableEntry TR;
    public NetworkTableEntry CT;

    public NetworkTableEntry tlong;
    public NetworkTableEntry tshort;

    public double v;
    public double a;
    public double x;
    public double y;
    public double area;
    public double ID;
    public double latency;
    public double tagLong;
    public double tagShort;

    public double[] bt; // botpose_targetspace
    public double[] cr;// camerapose_robotspace
    public double[] tr; // targetpose_robotpose;
    public double[] ct; // camerapose_targetspace

    public double MyDistance;

    public final double limelightLensHeightInches = 0;
    public final double limelightMountAngleDegrees = 0;
    public double targetOffsetAngle_Vertical;
    public double angleToGoalDegrees;
    public double angleToGoalRadians;
    public double goalHeightInches;

    public TagTrackingLimelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    public double getMyDistance() {
        // readValue();
        double target_height = getBT()[1]; // botpose in targetspace y
        double x_dis = getBT()[0];
        double z_dis = getBT()[2];
        double hori_dis = Math.pow(Math.pow(x_dis, 2) + Math.pow(z_dis, 2), 1.0 / 2);
        MyDistance = Math.pow(Math.pow(target_height, 2) + Math.pow(hori_dis, 2), 1.0
                / 2);

        return MyDistance;
    }

    public double getTx() {
        x = table.getEntry("tx").getDouble(0);
        return x;
    }

    public double getTy() {
        y = table.getEntry("ty").getDouble(0);
        return y;
    }

    public double getTa() {
        a = table.getEntry("ta").getDouble(0);
        return a;
    }

    public double getTv() {
        v = table.getEntry("tv").getDouble(0);
        return v;
    }

    public double getTID() {
        ID = table.getEntry("tid").getDouble(0);
        return ID;
    }

    public double getTl() {
        latency = table.getEntry("tl").getDouble(0);
        return latency;
    }

    public double[] getBT() {
        bt = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        return bt;
    }

    public double[] getCT() {
        ct = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        return ct;
    }

    public double getTlong(){
        tagLong = table.getEntry("tlong").getDouble(0);
        return tagLong;
    }

    public double getTshort(){
        tagShort = table. getEntry("tshort").getDouble(0);
        return tagShort;
    }

    // public void updatePoseEstimatorWithVisionBotPose() {
    //     PoseLatency visionBotPose = m_visionSystem.getPoseLatency();
    //     // invalid LL data
    //     if (visionBotPose.pose2d.getX() == 0.0) {
    //         return;
    //     }

    //     // distance from current pose to vision estimated pose
    //     double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
    //             .getDistance(visionBotPose.pose2d.getTranslation());

    //     if (m_visionSystem.areAnyTargetsValid()) {
    //         double xyStds;
    //         double degStds;
    //         // multiple targets detected
    //         if (m_visionSystem.getNumberOfTargetsVisible() >= 2) {
    //             xyStds = 0.5;
    //             degStds = 6;
    //         }
    //         // 1 target with large area and close to estimated pose
    //         else if (m_visionSystem.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
    //             xyStds = 1.0;
    //             degStds = 12;
    //         }
    //         // 1 target farther away and estimated pose is close
    //         else if (m_visionSystem.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
    //             xyStds = 2.0;
    //             degStds = 30;
    //         }
    //         // conditions don't match to add a vision measurement
    //         else {
    //             return;
    //         }

    //         m_poseEstimator.setVisionMeasurementStdDevs(
    //                 VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
    //         m_poseEstimator.addVisionMeasurement(visionBotPose.pose2d,
    //                 Timer.getFPGATimestamp() - visionBotPose.latencySeconds);
    //     }
    // }

    public void putDashboard() {
        // SmartDashboard.putNumber("hasTarget", getTv());
        SmartDashboard.putNumber("LimelightX", getTx());
        SmartDashboard.putNumber("LimelightY", getTy());
        // SmartDashboard.putNumber("LimelightArea", getTa());
        SmartDashboard.putNumber("LimelightID", getTID());
        SmartDashboard.putNumber("latency", getTl());

        // botpose in targetspace
        SmartDashboard.putNumber("bt_x", getBT()[0]);
        SmartDashboard.putNumber("bt_y", getBT()[1]);
        SmartDashboard.putNumber("bt_z", getBT()[2]);

        // campose in targetspace
        SmartDashboard.putNumber("ct_x", getCT()[0]);
        SmartDashboard.putNumber("ct_y", getCT()[1]);
        SmartDashboard.putNumber("ct_z", getCT()[2]);

        SmartDashboard.putNumber("MyDistance", getMyDistance());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        putDashboard();
    }
}
