package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;

public class AprilTag extends SubsystemBase {
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

    // Write to constants

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
        putDashboard();
    }

    public static void change_APipeline() {
        table.getEntry("pipeline").setNumber(AprilTagConstants.A_pipeline);
    }

    public static double InchesToMeter(double inches) {
        double meters = inches * 0.0254;
        return meters;
    }

    public static void readValue() {
        v = tv.getDouble(0);
        x = tx.getDouble(0.0);
        y = tx.getDouble(0.0);
        ID = tid.getDouble(0);
        latency = tl.getDouble(0.0);

        bt = BT.getDoubleArray(new double[6]);
        ct = CT.getDoubleArray(new double[6]);
        tr = TR.getDoubleArray(new double[6]);
        cr = CR.getDoubleArray(new double[6]);
    }

    public static double getMyDistance() {

        readValue();
        double target_height = bt[1]; // botpose in targetspace y
        double x_dis = bt[0];
        double z_dis = bt[2];
        double hori_dis = Math.pow(Math.pow(x_dis, 2) + Math.pow(z_dis, 2), 1.0 / 2);
        MyDistance = Math.pow(Math.pow(target_height, 2) + Math.pow(hori_dis, 2), 1.0 / 2);

        return MyDistance;
    }

    public static void putDashboard() {
        SmartDashboard.putNumber("hasTarget", v);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightID", ID);
        SmartDashboard.putNumber("latency", latency);

        // SmartDashboard.putNumberArray("bt", bt);
        // SmartDashboard.putNumberArray("ct", ct);
        // SmartDashboard.putNumberArray("cr", cr);
        // SmartDashboard.putNumberArray("tr", tr);

        // botpose in targetspace
        SmartDashboard.putNumber("bt_x", bt[0]);
        SmartDashboard.putNumber("bt_y", bt[1]);
        SmartDashboard.putNumber("bt_z", bt[2]);

        // campose in targetspace
        SmartDashboard.putNumber("ct_x", ct[0]);
        SmartDashboard.putNumber("ct_y", ct[1]);
        SmartDashboard.putNumber("ct_z", ct[2]);

        SmartDashboard.putNumber("current pipeline", table.getEntry("getpipe").getDouble(0));
        SmartDashboard.putNumber("MyDistance", MyDistance);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        putDashboard();
    }
}
