// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ApriltagTracking;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.proto.Geometry3D;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TagTrackingPhotovision extends SubsystemBase {
    /** Creates a new VisionTrackingPhotovision. */

    public AprilTagFieldLayout aprilTagFieldLayout;

    private final boolean tag = true;
    private final String cameraName = "Microsoft_LifeCam_HD-3000";
    private final PhotonCamera tagCamera;
    private final Transform3d tagCamPosition = new Transform3d(
            new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0)));

    public final double cameraHeight = 0.36;
    public final double cameraWeight = 0.0;
    public final double pitchDegree = 15; // 90 - cam_offset
    public final double yawDegree = 0;

    public PhotonPipelineResult results;
    public Transform3d robotToCam;
    // Construct PhotonPoseEstimator
    public PhotonPoseEstimator photonPoseEstimator;

    public double distance;
    public double yaw;
    public double pitch;
    public double x;
    public double y;
    public double ID;

    public TagTrackingPhotovision() {
        tagCamera = new PhotonCamera(cameraName);
        tagCamera.setPipelineIndex(0);
        tagCamera.setDriverMode(false);
        robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0, 0));
        // Cam mounted facing forward, half a metre forward of center, half a meter up
        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, tagCamera, robotToCam);

    }

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update();
    // }

    public List<Pose2d> getTags() {
        List<Pose2d> poses = new ArrayList<Pose2d>();

        if (!tagCamera.isConnected()) {
            System.out.println("Camera not connected");
        }

        results = tagCamera.getLatestResult();

        if (!results.hasTargets()) {
            System.out.println("No target");
        }

        List<PhotonTrackedTarget> targets = results.getTargets();

        for (PhotonTrackedTarget trackedTarget : targets) {
            // this calc assumes pitch angle is positive UP, so flip the camera's pitch
            pitch = trackedTarget.getPitch();
            yaw = trackedTarget.getYaw();
            y = cameraHeight * (1 / Math.tan(Math.toRadians(pitch - pitchDegree)));
            x = y * Math.tan(Math.toRadians(yaw - yawDegree)) + cameraWeight;
            poses.add(new Pose2d(x, y, new Rotation2d(0)));
        }
        return poses;
    }

    // not yet done
    public double getDistance() {
        // List<Double> distances = new ArrayList<Double>();
        distance = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight,
                y,
                Math.toRadians(pitch),
                Units.degreesToRadians(results.getBestTarget().getPitch()));
        return distance;
    }

    public void clearTagSolutions(Field2d field) {
        if (field == null)
            return;
        field.getObject("tagSolutions").setPoses();
        field.getObject("visionPose").setPoses();
        field.getObject("visionAltPose").setPoses();
        field.getObject("visibleTagPoses").setPoses();
    }

    public void plotPose(Field2d field, String label, Pose2d pose) {
        if (field == null)
            return;
        if (pose == null)
            field.getObject(label).setPoses();
        else
            field.getObject(label).setPose(pose);
    }

    // not done yet
    public Pose3d tagFieldPose3d() {
        Pose3d tagPose3d = new Pose3d();
        return tagPose3d;
    }

    // copy paste
    // public void convertCoordinateSystem() {

    // Transform3d tagT3D = Geometry3D.Translation3d(tagTranslation['x'],
    // tagTranslation['y'], tagTranslation['z']);
    // Rotation3d tagR3D = Geometry3D.Rotation3d(
    // Geometry3D.Quaternion(tagRotation['W'], tagRotation['X'], tagRotation['Y'],
    // tagRotation['Z']));
    // Pose3d tagPose3D = Geometry3D.Pose3d(tagT3D, tagR3D);

    // Pose3d tagToCameraTransform = AprilTagPoseEstimator.estimate(tag);

    // Pose3d wpiTranslation =
    // CoordinateSystem.convert(tagToCameraTransform.getTranslation().rotateBy(tagToCameraTransform.inverse().rotation()),
    // CoordinateSystem.EDN(),
    // CoordinateSystem.NWU());

    // // Pose3d cameraPose = tagPose3D.transformBy(wpiTransform.inverse());
    // }

    public double[] getTagInfo() {
        results = tagCamera.getLatestResult();
        double[] tagInfo = new double[2];
        tagInfo[0] = 0;
        tagInfo[1] = 0;
        // if (results.hasTargets()) {
        PhotonTrackedTarget target = results.getBestTarget();
        ID = target.getFiducialId();
        distance = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight,
                cameraHeight * (1 / Math.tan(Math.toRadians(target.getPitch() - pitchDegree))),
                Math.toRadians(target.getPitch()),
                Units.degreesToRadians(results.getBestTarget().getPitch()));
        // }
        tagInfo[0] = results.hasTargets() ? ID : 0;
        tagInfo[1] = results.hasTargets() ? distance : 0;
        return tagInfo;
    }

    public void putDashboard(){
        SmartDashboard.putNumber("distance", getTagInfo()[1]);
        SmartDashboard.putNumber("ID", getTagInfo()[0]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        List<Pose2d> tags = getTags();
        SmartDashboard.putNumber("tagVision/nFound", tags.size());
        if (tags.size() > 0) {
            Pose2d p = tags.get(0);
            SmartDashboard.putNumber("tagVision/x", p.getX());
            SmartDashboard.putNumber("tagVision/y", p.getY());
        }
    }
}
