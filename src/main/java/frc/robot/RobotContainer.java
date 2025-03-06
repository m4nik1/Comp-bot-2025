// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlgaeIn;
import frc.robot.commands.CoralIn;
import frc.robot.commands.CoralOut;
import frc.robot.commands.RunCoralIntake;
import frc.robot.commands.RunElevatorManual;
import frc.robot.commands.RunPivotManual;
import frc.robot.commands.SetCoralPivot;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public static DriveTrain driveTrain = new DriveTrain();
  public static Elevator elevator = new Elevator();
  public static CoralIntake coralIntake = new CoralIntake();
  public static CoralPivot coralPivot = new CoralPivot();
  public static AlgaeIntake algaeIntake = new AlgaeIntake();

  static CommandXboxController driver = new CommandXboxController(0);
  static CommandXboxController operator = new CommandXboxController(1);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    driveTrain.setDefaultCommand(new TeleopDrive());
    elevator.setDefaultCommand(new RunElevatorManual());
    // coralIntake.setDefaultCommand(new RunCoralIntake());
    coralPivot.setDefaultCommand(new RunPivotManual());
    configureBindings();
  }

  private void configureBindings() {
    // operator.a().onTrue(new SetCoralPivot());
    operator.a().whileTrue(new CoralOut());
    operator.b().whileTrue(new CoralIn());
    operator.y().whileTrue(new AlgaeIn());
  }

  public static double getLeftYOp() {
    return operator.getLeftY();
  }

  
  public static double getRightYOp() {
    return operator.getRightY();
  }

  public static double getLeftY() {
    return driver.getLeftY();
  }

  public static double getRightX() {
    return driver.getRightX();
  }

  public static double getLeftX() {
    return driver.getLeftX();
  }
}
