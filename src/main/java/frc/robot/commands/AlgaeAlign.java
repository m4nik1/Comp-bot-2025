// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAlign extends Command {
  /** Creates a new CoralLeftAlign. */
  public AlgaeAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.photonVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var results = RobotContainer.photonVision.getUnreadResults();
    if(!results.isEmpty()) {
      
      // Gets the latest frame since one has been processed since then
      var result = results.get(results.size() - 1);
      if(result.hasTargets()) { // At least one tag has been seen by the camera
        for (var target : result.getTargets()) {
          // For now it will be 17 in front of the reef
          if(target.getFiducialId() == 17) {
            
          }
        }
      }
    }

    // RobotContainer.driveTrain.drive(new Translation2d(), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
