// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  public static DriveTrain driveTrain = new DriveTrain();
  public static Elevator elevator = new Elevator();

  public static CommandXboxController controller = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    driveTrain.setDefaultCommand(new TeleopDrive());
  }

  private void configureBindings() {

  }

  public static double getLeftY() {
    return controller.getLeftY();
  }

  public static double getRightX() {
    return controller.getRightX();
  }

  public static double getLeftX() {
    return controller.getLeftX();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
  // }
}
