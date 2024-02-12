// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ApriltagCmd.FaceMethod2Cmd;
import frc.robot.commands.ApriltagCmd.FaceTag;
import frc.robot.commands.ApriltagCmd.FixDistanceCmd;
import frc.robot.commands.ApriltagCmd.FollowCmd;
import frc.robot.commands.ApriltagCmd.Go45andFaceCmd;
import frc.robot.commands.TrackingCmd.AddTrackingError;
import frc.robot.commands.TrackingCmd.MinusTrackingError;
import frc.robot.commands.TrackingCmd.SwitchTrackConditionCmd;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  Drivebase drivetain;
  AprilTag tag;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final PowerDistribution pd = new PowerDistribution();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    putDashboard();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    drivetain = new Drivebase();
    tag = new AprilTag();
    drivetain.setDefaultCommand(new SwerveJoystickCmd(drivetain, driverController));
    driverController.x().onTrue(new SwitchTrackConditionCmd(drivetain));
    driverController.a().toggleOnTrue(new FaceTag(tag, drivetain));
    driverController.b().toggleOnTrue(new FixDistanceCmd(drivetain));
    driverController.y().toggleOnTrue(new FollowCmd());
  driverController.back().toggleOnTrue(new FaceMethod2Cmd(drivetain));
    driverController.pov(45).toggleOnFalse(new Go45andFaceCmd()); // wonder if we should use on true
    driverController.pov(90).onTrue(new AddTrackingError(drivetain));
    driverController.pov(270).onTrue(new MinusTrackingError(drivetain));
  }

  private void putDashboard() {
    SmartDashboard.putNumber("xbox_leftX", driverController.getLeftX());
    SmartDashboard.putNumber("xbox_leftY", driverController.getLeftY());
    SmartDashboard.putNumber("pd_voltage", pd.getVoltage());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}
