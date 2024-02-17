// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteTrackingPhotovision extends SubsystemBase {
  /** Creates a new VisionTrackingPhotovision. */

  private final boolean plotNote = true;
    private final String cameraName = "Microsoft_LifeCam_HD-3000";
    private final PhotonCamera noteCamera;
    private final Transform3d noteCamPosition = new Transform3d(
            new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0)));

    public final double cameraHeight = 0.36;
    public final double cameraWeight = 0.0;
    public final double pitchDegree = 0;
    public final double yawDegree = 0;


    public NoteTrackingPhotovision() {
        noteCamera = new PhotonCamera(cameraName);
        noteCamera.setPipelineIndex(1);
        noteCamera.setDriverMode(false);
    }

    public List<Pose2d> getNotes() {
        List<Pose2d> poses = new ArrayList<Pose2d>();

        if (!noteCamera.isConnected()) {
            System.out.println("Camera not connected");
            return poses;
        }

        var results = noteCamera.getLatestResult();
        if (!results.hasTargets()) {
            System.out.println("No target");
            return poses;
        }
        List<PhotonTrackedTarget> targets = results.getTargets();

        for (PhotonTrackedTarget trackedTarget : targets) {
            // this calc assumes pitch angle is positive UP, so flip the camera's pitch
            // note that PV target angles are in degrees
            // double d = Math.abs(noteCamPosition.getZ() /
            //         Math.tan(-noteCamPosition.getRotation().getY() + Math.toRadians(trackedTarget.getPitch())));
            // double yaw = Math.toRadians(trackedTarget.getYaw());
            // double x = d * Math.cos(yaw);
            // double y = d * Math.sin(yaw);
            double pitch = trackedTarget.getPitch();
            double yaw = trackedTarget.getYaw();
            double y = cameraHeight*(1/Math.tan(Math.toRadians(pitch-pitchDegree)));
            double x = y*Math.tan(Math.toRadians(yaw-yawDegree))+cameraWeight;
            poses.add(new Pose2d(x, y, new Rotation2d(0)));
        }
        return poses;
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

    public void periodic() {
        // This method will be called once per scheduler run
        List<Pose2d> notes = getNotes();
        SmartDashboard.putNumber("noteVision/nFound", notes.size());
        if (notes.size() > 0) {
            Pose2d p = notes.get(0);
            SmartDashboard.putNumber("noteVision/x", p.getX());
            SmartDashboard.putNumber("noteVision/y", p.getY());
        }
    }
}
