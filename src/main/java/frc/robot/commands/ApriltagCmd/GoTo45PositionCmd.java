// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ApriltagCmd;

// import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivebase;

public class GoTo45PositionCmd extends Command {
  /** Creates a new FixDistanceCmd. */
  // private final AprilTag tag;
  private final Drivebase drivebase;

  public GoTo45PositionCmd(Drivebase drivebase) {
    // this.tag = tag;
    this.drivebase = drivebase;
    addRequirements(drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.Go_To_45_Angle();
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
