// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ApriltagCmd;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.Drivebase;

public class AprilTagFollow extends Command {
  /** Creates a new AprilTagFollow. */
  private final Drivebase drivebase;
  private final AprilTag aprilTag;
  private final CommandXboxController main;
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3.0);
  public double speed;

  public AprilTagFollow(Drivebase drivebase, AprilTag aprilTag, CommandXboxController main) {
    this.aprilTag = aprilTag;
    this.drivebase = drivebase;
    this.main = main;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase, aprilTag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aprilTag.init();
    aprilTag.change_APipeline();
    speed = speedLimiter.calculate(main.getLeftY() * 5.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.trackingDrive(aprilTag.getTx(), speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
