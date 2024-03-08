// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.apriltagTracking;

import java.io.IOException;
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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class TagTrackingPhotonvision extends SubsystemBase {
    /** Creates a new VisionTrackingPhotovision. */

    private double m_latestLatency;
    public AprilTagFieldLayout aprilTagFieldLayout;

    private final boolean tag = true;
    private final String cameraName = "Microsoft_LifeCam_HD-3000";
    private final PhotonCamera tagCamera;
    // private final Transform3d tagCamPosition = new Transform3d(
    // new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0),
    // Units.inchesToMeters(0.0)),
    // new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0),
    // Math.toRadians(0.0)));

    public final double cameraHeight = 0.14;
    public final double cameraWeight = 0.0; // I'm still quite confused abt the meaning of having this
    public final double pitchDegree = 10; // 90 - cam_offset
    public final double yawDegree = 0;
    public final double cameraPitch = 15;
    public final double targetHeight = 1.3;

    public PhotonPipelineResult results;
    public Transform3d robotToCam = VisionConstants.krobottocam;
    public AprilTagFieldLayout m_layout;
    // Construct PhotonPoseEstimator
    public PhotonPoseEstimator photonPoseEstimator;

    public double distance;
    public double yaw;
    public double pitch;
    public double x;
    public double y;
    public double ID;
    public boolean hasTarget;

    public TagTrackingPhotonvision() {
        tagCamera = new PhotonCamera(cameraName);
        tagCamera.setPipelineIndex(0);
        tagCamera.setDriverMode(false);
        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, tagCamera, robotToCam);

        try {
            m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException err) {
            throw new RuntimeException();
        }
    }

    /**
     * Returns pipeline result
     * 
     * @return (@link PhotonPipelineResult} results
     */
    public PhotonPipelineResult getPipelineResult() {
        results = tagCamera.getLatestResult();
        return results;
    }

    /**
     * Whether the tag's detected
     * 
     * @return hasTarget
     */
    public boolean hasTarget() {
        hasTarget = getPipelineResult().hasTargets();
        return hasTarget;
    }

    /**
     * Get Best Target according to the pipeline.
     * 
     * @return {@link PhotonTrackedTarget} best target
     */
    public PhotonTrackedTarget getBestTarget() {
        results = getPipelineResult();
        m_latestLatency = results.getLatencyMillis() / 1000.;
        PhotonTrackedTarget target = null;
        if (results.hasTargets()) {
            target = results.getBestTarget();
        }
        return target;
    }

    /**
     * Get a list of tracked tags according to the pipeline.
     * 
     * @return a list of tracked {@link PhotonTrackedTarget} tags
     */
    public List<PhotonTrackedTarget> getTargets() {
        // results = getPipelineResult();
        List<PhotonTrackedTarget> tags = getPipelineResult().getTargets();
        return tags;
    }

    /**
     * Returns a list of 2 demesional tag poses
     * 
     * @return {@link List}<{@link Pose2d}> poses
     */
    public List<Pose2d> getTags() {
        List<Pose2d> poses = new ArrayList<Pose2d>();

        double tagHeight = getTagPose3d().getY();
        List<PhotonTrackedTarget> targets = results.getTargets();

        // for(type var : arr) basically put each value of array arr into var, one by a
        // time
        for (PhotonTrackedTarget trackedTarget : targets) {
            // this calc assumes pitch angle is positive UP, so flip the camera's pitch
            pitch = trackedTarget.getPitch();
            yaw = trackedTarget.getYaw();
            y = tagHeight * (1 / Math.tan(Math.toRadians(pitch + pitchDegree)));
            x = y * Math.tan(Math.toRadians(yaw - yawDegree)) + cameraWeight;
            poses.add(new Pose2d(x, y, new Rotation2d(0)));
        }
        return poses;
    }

    /**
     * Return the last tag of the getTargets() list
     * 
     * @return {@link PhotontrackedTarget} last tag
     */
    public PhotonTrackedTarget getLastTag() {
        return getTargets().get(0);
    }

    /**
     * Return the last tag of the getTargets() list
     * 
     * @return {@link Pose2d} last tag pose
     */
    public Pose2d getLastPose() {
        return getTags().get(0);
    }

    public double[] getTagInfo2() {
        double[] tagInfo = new double[5];
        PhotonTrackedTarget target = getBestTarget();
        ID = target.getFiducialId();
        // distance = PhotonUtils.calculateDistanceToTargetMeters(
        //         cameraHeight, // height
        //         cameraHeight * (1 / Math.tan(Math.toRadians(target.getPitch() + pitchDegree))),
        //         Math.toRadians(target.getPitch()),
        //         Units.degreesToRadians(results.getBestTarget().getPitch()));
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();              
        double range = getBestTagTransform3d().getX()*Math.cos(Math.toRadians(cameraPitch + pitch));
        // Transform3d pose = target.getBestCameraToTarget();
        // List<TargetCorner> corners = target.getDetectedCorners();
        tagInfo[0] = ID;
        tagInfo[1] = range;
        tagInfo[2] = yaw;
        tagInfo[3] = pitch;
        tagInfo[4] = area;
        return tagInfo;
    }

    /**
     * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
     * object/fiducial tag space (X forward, Y left, Z up) with the lowest
     * reprojection error
     * 
     * @return {@link Transform3d} best cam to tag
     */
    public Transform3d getTagTransform3d() {
        Transform3d pose = getLastTag().getBestCameraToTarget();
        return pose;
    }

    /**
     * Get best transform that maps camera space (X = forward, Y = left, Z = up) to
     * object/fiducial tag space (X forward, Y left, Z up) with the lowest
     * reprojection error
     * 
     * @return {@link Transform3d} best cam to tag
     */
    public Transform3d getBestTagTransform3d() {
        Transform3d pose = results.getBestTarget().getBestCameraToTarget();
        return pose;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public int getTagID() {
        int ID = getBestTarget().getFiducialId();
        return ID;
    }

    // public double getTagTx(){
    // double tx = getBestTarget().getTargetPixelsX();
    // return tx;
    // }

    public Pose3d getBotPose() {
        if (getBestTarget() != null) {
            Transform3d camToTag = getBestTarget().getBestCameraToTarget();
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose(getTagID());
            if (tag_Pose3d.isPresent()) {
                Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(camToTag, tag_Pose3d.get(), robotToCam);
                return robotPose3d;
            }
        }
        return new Pose3d();
    }

    // well any way let's just leave it like that ^^
    // repeat. same as getTagInfo()[1]
    public double getDistance() {
        // List<Double> distances = new ArrayList<Double>();
        distance = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight,
                y,
                Math.toRadians(pitch),
                Units.degreesToRadians(results.getBestTarget().getPitch()));
        return distance;
    }

    // used in facePhoton() method in drivebase
    public Rotation2d getYawToPoseRotation2d(Pose2d robotPose, Pose2d targetPose) {
        Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose);
        return targetYaw;
    }

    // 
    public Pose2d getLatestEstimatedRobotPose() {
        
        if (hasTarget()) {
            Transform3d cameraToTarget = getBestTarget().getBestCameraToTarget();

            Optional<Pose3d> tagPose = m_layout.getTagPose(getBestTarget().getFiducialId());
            // Use Optional so we don't need to do null check

            Transform3d camToRobot = new Transform3d();

            if (tagPose.isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);
                return robotPose.toPose2d();
            }
        }
        return new Pose2d();
    }

    /**
     * Gets the pose of a target.
     * 
     * @param robotPose The current robot pose.
     * @param offset    The offset of the desired pose from the target. Positive is
     *                  backwards (X) and right (Y).
     * @return The pose of the specified offset from the target.
     */
    public Pose2d getTargetPose(Pose2d robotPose, Transform3d offset) {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Transform3d targetOffset = cameraToTarget.plus(offset);

            Pose3d pose = new Pose3d(robotPose);
            // Constructs a 3D pose from a 2D pose in the X-Y plane

            Pose3d scoringPose = pose.plus(targetOffset);

            // WARNING: The following code is scuffed. Please proceed with caution.
            Pose2d newPose = scoringPose.toPose2d();

            Rotation2d newRotation = Rotation2d.fromDegrees(newPose.getRotation().getDegrees() - 180.);

            Pose2d finalPose = new Pose2d(newPose.getTranslation(), newRotation).plus(
                    new Transform2d(
                            VisionConstants.krobottocam.getTranslation().toTranslation2d(),
                            VisionConstants.krobottocam.getRotation().toRotation2d()));
            return finalPose;
        }

        return robotPose;
    }

    public double getLatestLatency() {
        return m_latestLatency;
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

    /**
     * Gets the best tag's pose in 2 dimension
     * 
     * @return best tagPose
     */
    public Pose2d getTagPose2d() {
        if (hasTarget()) {
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose(getTagID());
            Pose2d tagPose2d = tag_Pose3d.isPresent() ? tag_Pose3d.get().toPose2d() : new Pose2d();
            return tagPose2d;
        } else {
            return new Pose2d();
        }
    }
    
    public Pose3d getTagPose3d() {
        if (hasTarget()) {
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose(getTagID());
            Pose3d tagPose3d = tag_Pose3d.isPresent() ? tag_Pose3d.get() : new Pose3d();
            return tagPose3d;
        } else {
            return new Pose3d();
        }
    }

    /**
     * Get the pose of the desired tag in 2 dimension
     * 
     * @param index tag ID
     * @return best tagPose
     */
    public Pose2d getDesiredTagPose(int index) {
        if (hasTarget()) {
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose(index);
            Pose2d tagPose2d = tag_Pose3d.isPresent() ? tag_Pose3d.get().toPose2d() : new Pose2d();
            return tagPose2d;
        } else {
            return new Pose2d();
        }
    }

    public void putDashboard() {
        SmartDashboard.putNumber("ID", getTagInfo2()[0]);
        SmartDashboard.putNumber("range", getTagInfo2()[1]);
        SmartDashboard.putNumber("yaw", getTagInfo2()[2]);
        SmartDashboard.putNumber("pitch", getTagInfo2()[3]);
        SmartDashboard.putNumber("area", getTagInfo2()[4]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        List<Pose2d> tags = getTags();
        SmartDashboard.putNumber("tagVision/nFound", tags.size());
        if (tags.size() > 0) {
            SmartDashboard.putNumber("tagVision/x", getBestTagTransform3d().getX());
            SmartDashboard.putNumber("tagVision/y", getBestTagTransform3d().getY());
        }
    }
}
