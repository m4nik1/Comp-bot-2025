// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralPivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunPivotManual extends Command {
  /** Creates a new RunPivotManual. */

  boolean stop = false;
  public RunPivotManual() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.coralPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.coralPivot.runPivotManual(-RobotContainer.getRightYOp());

    // if(RobotContainer.coralPivot.getPivotLimit()) {
    //   stop = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the Pivot speed
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
