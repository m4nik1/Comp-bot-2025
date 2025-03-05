// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.RunCoralIntake;
import frc.robot.commands.RunElevatorManual;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public static DriveTrain driveTrain = new DriveTrain();
  public static Elevator elevator = new Elevator();
  public static CoralIntake coralIntake = new CoralIntake();

  static CommandXboxController controller = new CommandXboxController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    driveTrain.setDefaultCommand(new TeleopDrive());
    // elevator.setDefaultCommand(new RunElevatorManual());
    // coralIntake.setDefaultCommand(new RunCoralIntake());
    configureBindings();
  }

  private void configureBindings() {
  }

  public static double getLeftYOp() {
    return controller.getLeftY();
  }

  
  public static double getRightYOp() {
    return controller.getLeftY();
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
}
