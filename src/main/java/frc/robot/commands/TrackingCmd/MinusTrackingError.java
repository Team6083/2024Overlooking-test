// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackingCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class MinusTrackingError extends Command {
  /** Creates a new MinusTrackingError. */
  private final Drivebase drivetain;

  public MinusTrackingError(Drivebase drivetain) {
    this.drivetain = drivetain;
    addRequirements(drivetain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetain.minusTrackTargetError();
  }
}
