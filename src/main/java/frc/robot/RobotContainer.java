// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ApriltagCmd.FaceTag;
// import frc.robot.commands.ApriltagCmd.FollowCmd;
// import frc.robot.commands.TrackingCmd.AddTrackingError;
// import frc.robot.commands.TrackingCmd.MinusTrackingError;
import frc.robot.commands.TrackingCmd.SwitchTrackConditionCmd;
import frc.robot.subsystems.ApriltagTracking.TagTrackingLimelight;
import frc.robot.subsystems.drive.Drivebase;
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
  Drivebase drivebase;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController mainController = new CommandXboxController(
      OperatorConstants.kMainControllerPort);

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
    drivebase = new Drivebase();
    drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, mainController));
    // mainController.y().onTrue(new FaceTag(drivebase).withTimeout(0.5).andThen(new FollowCmd(drivebase).withTimeout(2)));;
    mainController.x().onTrue(new SwitchTrackConditionCmd(drivebase));
    // driverController.pov(90).onTrue(new AddTrackingError(drivetain));
    // driverController.pov(270).onTrue(new MinusTrackingError(drivetain));
  }

  private void putDashboard() {
    SmartDashboard.putNumber("xbox_leftX", mainController.getLeftX());
    SmartDashboard.putNumber("xbox_leftY", mainController.getLeftY());
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
