// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RunCoralIntake;
import frc.robot.commands.RunElevator;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DriveTrainKrakens;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  public static DriveTrainKrakens driveTrain = new DriveTrainKrakens();
  public static Elevator elevator = new Elevator();
  public static CoralIntake coralIntake = new CoralIntake();
  public static AlgaeIntake algaeIntake = new AlgaeIntake();

  static CommandXboxController controller = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // elevator.setDefaultCommand(new RunElevator());
    coralIntake.setDefaultCommand(new RunCoralIntake());
  }

  private void configureBindings() {

  }

  public static double getY() {
    return controller.getLeftY();
  }

  public static double getRightY() {
    return controller.getRightY();
  }


  // public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
