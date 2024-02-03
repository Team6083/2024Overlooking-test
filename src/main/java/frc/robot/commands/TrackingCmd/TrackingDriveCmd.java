// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackingCmd;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.VisionTracking;

public class TrackingDriveCmd extends Command {
  /** Creates a new TrackingDriveCmd. */
  private final Drivebase drivebase;
  private final CommandXboxController main;
  private final VisionTracking visionTracking = new VisionTracking();
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3.0);
  private double speed;

  public TrackingDriveCmd(Drivebase drivebase, CommandXboxController main) {
    this.drivebase = drivebase;
    this.main = main;
    addRequirements(this.drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = speedLimiter.calculate(main.getLeftY() * 5.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.trackingDrive(visionTracking.getTx(), speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
