// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivebase;

public class GyroresetCmd extends Command {
  /** Creates a new GyroresetCmd. */
private final Drivebase drivebase;
  public GyroresetCmd(Drivebase drivebase) {
// Use addRequirements() here to declare subsystem dependencies.
  this.drivebase = drivebase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        drivebase.setGyroReset();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
