// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorAuto_l4 extends Command {
  /** Creates a new ElevatorAuto_l4. */

  boolean stop = false;

  Timer timer;

  public ElevatorAuto_l4() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.reset();
    RobotContainer.elevator.setElevatorMagic(Constants.elevator_l4);

    if(RobotContainer.elevator.isElevatorL4()) {
      RobotContainer.coralPivot.setPivot(Constants.pivot_down);
      timer.start();
      if(timer.hasElapsed(0.2)) {
        RobotContainer.coralIntake.coralOut();
      }
    }
    if(timer.hasElapsed(1)) {
      stop = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
