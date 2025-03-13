// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAlign extends Command {

  Pose2d algaePose;

  public AlgaeAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.photonVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // var results = RobotContainer.photonVision.getUnreadResults();
    // if(!results.isEmpty()) {
      
    //   // Gets the latest frame since one has been processed since then
    //   var latestResult = results.get(results.size() - 1);
    //   if(latestResult.hasTargets()) { // At least one tag has been seen by the camera
    //     for (var target : latestResult.getTargets()) {
    //       int tagId = target.getFiducialId();

    //       // Finds the tags that are associated with the reef
    //       if(Constants.desiredTagIds.contains(tagId)) {
    //         double face = Constants.TagToFaceBlue.get(tagId);
            
    //         // Calculates the face angle in radians
    //         double thetaCalculate = ((2*Math.PI)*(face/6)+Math.PI) % (2*Math.PI); 

    //         Transform2d calculatedAlgae = Robot.reefPosesGenerate.calculateAlgaePose(thetaCalculate);

    //         algaePose = RobotContainer.driveTrain.getRobotPose2d().transformBy(calculatedAlgae);
    //       }
    //     }
    //   }
    // }

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
