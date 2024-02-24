// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.NoteTracking;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteTrackingConstants;

public class NoteTrackingPhotovision extends SubsystemBase {
    /** Creates a new VisionTrackingPhotovision. */

    private final boolean driveMode = false;
    private final PhotonCamera noteCamera;

    public NoteTrackingPhotovision() {
        noteCamera = new PhotonCamera(NoteTrackingConstants.cameraName);
        noteCamera.setPipelineIndex(NoteTrackingConstants.noteTrakingPipeline);
        noteCamera.setDriverMode(driveMode);
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
            double pitch = trackedTarget.getPitch(); // look down
            double yaw = trackedTarget.getYaw(); // right and left
            double y = NoteTrackingConstants.cameraHeight
                    * (1 / Math.tan(Math.toRadians(pitch - NoteTrackingConstants.pitchDegree)));
            double x = y * Math.tan(Math.toRadians(yaw - NoteTrackingConstants.yawDegree))
                    - NoteTrackingConstants.cameraWeight;
            poses.add(new Pose2d(x, y, new Rotation2d(0)));
            SmartDashboard.putNumber("noteYaw", trackedTarget.getYaw());
            SmartDashboard.putNumber("notePitch", trackedTarget.getPitch());
            SmartDashboard.putNumber("noteSkew", trackedTarget.getSkew());
        }
        return poses;
    }

    public Pose2d getLastPose() {
        return getNotes().get(0);
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

    @Override
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
