// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackingCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivebase;
import frc.robot.subsystems.drive.SwerveModule;

public class MoveToSpecificPointCmd extends Command {
  /** Creates a new MoveToSpecificPoint. */
  Drivebase drivebase;
  double Zsetpoint;
  double Xsetpoint;
  double rotSetPoint;
  public MoveToSpecificPointCmd(Drivebase drivebase, double Zsetpoint,double Xsetpoint,double rotSetPoint) {
    this.drivebase = drivebase;
    this.Xsetpoint = Xsetpoint;
    this.Zsetpoint = Zsetpoint;
    this.rotSetPoint = rotSetPoint;
    addRequirements(this.drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Xspeed = drivebase.moveToSpecificPoint(Zsetpoint,Xsetpoint)[0];
    double Yspeed = drivebase.moveToSpecificPoint(Zsetpoint,Xsetpoint)[1]; 
    double rot = drivebase.autosetRotation(rotSetPoint);
    drivebase.drive(Xspeed, Yspeed, rot, false);
    
    

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
