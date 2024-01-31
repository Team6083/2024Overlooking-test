package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTag {
    public static NetworkTable table;
    public static NetworkTableEntry tx;// = table.getEntry("tx");// table.getEntry("tx");
    public static NetworkTableEntry ty;// = table.getEntry("ty");
    public static NetworkTableEntry ta;// = table.getEntry("ta");
    public static NetworkTableEntry tv;
    public static NetworkTableEntry tid;
    public static NetworkTableEntry tl;

    public static NetworkTableEntry BT;
    public static NetworkTableEntry CR;
    public static NetworkTableEntry TR;
    public static NetworkTableEntry CT;

    public static double v;
    public static double x;
    public static double y;
    public static double area;
    public static double ID;
    // public static double cl;
    public static double latency;

    public static double[] bt; // botpose_targetspace
    public static double[] cr;// camerapose_robotspace
    public static double[] tr; // targetpose_robotpose;
    public static double[] ct; // camerapose_targetspace

    public static double distance;
    public static double MyDistance;

    public static final double limelightLensHeightInches = 0;
    public static final double limelightMountAngleDegrees = 0;
    public static double goalHeightInches;

    public static void init() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");// table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");
        tl = table.getEntry("tl");

        BT = table.getEntry("botpose_targetspace");
        CR = table.getEntry("camerapose_robotspace");
        TR = table.getEntry("targepose_robotspace");
        CT = table.getEntry("camerapose_targetspace");

    }

    public static void loop() {
        // readValue();
        v = tv.getDouble(0);
        x = tx.getDouble(0.0);
        y = tx.getDouble(0.0);
        ID = tid.getDouble(0);
        latency = tl.getDouble(0.0);

        bt = BT.getDoubleArray(new double[6]);
        ct = CT.getDoubleArray(new double[6]);
        tr = TR.getDoubleArray(new double[6]);
        cr = CR.getDoubleArray(new double[6]);

        // distance = getDistance();
        // MyDistance = getMyDistance();
        putDashboard();
    }

    /*
     * public static void readValue() {
     * v = tv.getDouble(0);
     * x = tx.getDouble(0.0);
     * y = tx.getDouble(0.0);
     * ID = tid.getDouble(0);
     * latency = tl.getDouble(0.0);
     * }
     */

    // public static double getDistance() {
    //     double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    //     goalHeightInches = bt[1];
    //     // distance from the target to the floor
    //     double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    //     double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    //     distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    //     // goalheightinches read pose
    //     return distance;
    // }

    // public static double getMyDistance() {
    //     // double a = -1;
    //     // double b = (a > 0) ? a : -a;

    //     double target_height = bt[1]; // botpose in targetspace y
    //     double x_dis = bt[0];
    //     double z_dis = bt[2];
    //     double hori_dis = Math.pow(Math.pow(x_dis, 2) + Math.pow(z_dis, 2), 1.0 / 2);
    //     MyDistance = Math.pow(Math.pow(target_height, 2) + Math.pow(hori_dis, 2), 1.0 / 2);

    //     return MyDistance;
    // }

    public static void putDashboard() {
        SmartDashboard.putNumber("hasTarget", v);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightID", ID);
        SmartDashboard.putNumber("latency", latency);

        SmartDashboard.putNumberArray("bt", bt);
        SmartDashboard.putNumberArray("ct", ct);
        SmartDashboard.putNumberArray("cr", cr);
        SmartDashboard.putNumberArray("tr", tr);


        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("MyDistance", MyDistance);
    }
}

