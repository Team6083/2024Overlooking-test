// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackingCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivebase;

public class SwitchTrackConditionCmd extends Command {
  /** Creates a new SwitchTrackConditionCmd. */
  private final Drivebase drivetain;

  public SwitchTrackConditionCmd(Drivebase drivetain) {
    this.drivetain = drivetain;
    addRequirements(this.drivetain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetain.switchTrackCondition();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
