// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCoralIntakeAuto extends Command {
  /** Creates a new RunCoralIntakeAuto. */

  Timer timer;
  boolean finished;

  public RunCoralIntakeAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    if(timer.hasElapsed(1.5)) {
      finished = true;
    }
    RobotContainer.coralIntake.runCoral(.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
