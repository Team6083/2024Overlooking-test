// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackingCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTag;

public class AprilTagDetect extends Command {
  /** Creates a new AddTrackingError. */
  private final AprilTag apriltag;

  public AprilTagDetect(AprilTag apriltag) {
    this.apriltag = apriltag;
    addRequirements(this.apriltag);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    apriltag.init();
    apriltag.change_APipeline();
  }

  @Override
  public void execute(){
    apriltag.loop();
  }

  @Override
  public void end(boolean interrupted){

  }

  @Override
  public boolean isFinished(){
    return false;
  }


}
