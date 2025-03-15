// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberStartSet extends Command {
  /** Creates a new ClimberStartSet. */

  boolean press = false;
  public ClimberStartSet() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    press = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    press = true;
    Logger.recordOutput("Climber set pressed", press);
    RobotContainer.climber.climbPosition(.617);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    press = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
