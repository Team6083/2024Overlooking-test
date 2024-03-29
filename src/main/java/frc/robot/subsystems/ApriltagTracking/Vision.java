// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/Team4028/2023-Drive/blob/alpha_auto/src/main/java/frc/robot/subsystems/Vision.java#L91

package frc.robot.subsystems.apriltagTracking;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private PhotonCamera m_camera;
    private AprilTagFieldLayout m_layout;

    private double m_latestLatency;

    private static final Pose3d ROBOT_TO_CAMERA = new Pose3d(Units.inchesToMeters(12.), Units.inchesToMeters(2.), 0.,
            new Rotation3d(0., Units.degreesToRadians(56.), Units.degreesToRadians(0.)));

    private static Vision m_instance;

    /** Creates a new Vision. */
    public Vision() {
        m_camera = new PhotonCamera(VisionConstants.kCameraName);

        try {
            m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException err) { 
            //IOException is the base class for exceptions thrown while accessing information using streams, files and directories
            throw new RuntimeException();
        }
    }

    /**
     * Get the best target from the camera.
     * 
     * @return The target data, or null if none are found.
     */
    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = m_camera.getLatestResult();

        m_latestLatency = result.getLatencyMillis() / 1000.;

        boolean hasTarget = result.hasTargets();

        PhotonTrackedTarget target = null;

        if (hasTarget) {
            target = result.getBestTarget();
        }
        return target;
    }

    public Pose2d getLatestEstimatedRobotPose() {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Optional<Pose3d> tagPose = m_layout.getTagPose(target.getFiducialId());
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
                            ROBOT_TO_CAMERA.getTranslation().toTranslation2d(),
                            ROBOT_TO_CAMERA.getRotation().toRotation2d()));
            return finalPose;
        }

        return robotPose;
    }

    public double getLatestLatency() {
        return m_latestLatency;
    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}