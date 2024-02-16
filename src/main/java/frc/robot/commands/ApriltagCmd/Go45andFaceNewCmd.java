// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ApriltagCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Go45andFaceNewCmd extends SequentialCommandGroup {
  /** Creates a new Go45andFaceNewCmd. */
  public Go45andFaceNewCmd(Drivebase drivebase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FaceTag(drivebase).withTimeout(0.5), new FixDistanceCmd(drivebase));
  }
}
