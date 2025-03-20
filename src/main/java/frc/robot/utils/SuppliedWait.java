// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SuppliedWait extends Command {
  /** Creates a new SuppliedWait. */
  private Timer timer = new Timer();
  Supplier<Double> duration;

  public SuppliedWait(Supplier<Double> seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    duration = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration.get());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Adds duration to the dashboard for debugging
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Duration", () -> duration.get(), null);
  }
}
