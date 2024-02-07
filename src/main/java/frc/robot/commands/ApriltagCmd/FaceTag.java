// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ApriltagCmd;

// import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.Drivebase;

public class FaceTag extends Command {
  /** Creates a new FaceTag. */
  public AprilTag aprilTag;
  public Drivebase drivebase;
  public FaceTag(AprilTag aprilTag, Drivebase drivebase) {
    this.aprilTag = aprilTag;
    this.drivebase = drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(aprilTag, drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // aprilTag.init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  drivebase.faceTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
