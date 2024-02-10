package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.AprilTagConstants;

public class AprilTag extends SubsystemBase {
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

    public double v;
    public double a;
    public double x;
    public double y;
    public double area;
    public double ID;
    public double latency;

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

    public void init() {
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
        init();
        x = table.getEntry("tx").getDouble(0);
        return x;
    }

    public double getTy() {
        init();
        y = table.getEntry("ty").getDouble(0);
        return y;
    }

    public double getTa() {
        init();
        a = table.getEntry("ta").getDouble(0);
        return a;
    }

    public double getTv() {
        init();
        v = table.getEntry("tv").getDouble(0);
        return v;
    }

    public double getTID() {
        init();
        ID = table.getEntry("tid").getDouble(0);
        return ID;
    }

    public double getTl() {
        init();
        latency = table.getEntry("tl").getDouble(0);
        return latency;
    }

    public double[] getBT() {
        init();
        bt = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        return bt;
    }

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
        // SmartDashboard.putNumber("ct_x", getCT()[0]);
        // SmartDashboard.putNumber("ct_y", getCT()[1]);
        // SmartDashboard.putNumber("ct_z", getCT()[2]);

        SmartDashboard.putNumber("MyDistance", getMyDistance());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        putDashboard();
    }
}
