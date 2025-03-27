// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AlgaeAlign extends Command {

  Pose2d algaePose;

  PIDController xTranslation, yTranslation, rotation;

  double translationVal, strafeVal, rotationVal;
  double xOutput, yOutput;

  public AlgaeAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.photonVision);
    addRequirements(RobotContainer.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xTranslation = new PIDController(.5, 0, 0);
    yTranslation = new PIDController(.5, 0, 0);
    rotation = new PIDController(.5, 0, 0);
    rotation.enableContinuousInput(-Math.PI, Math.PI);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      xTranslation.reset();
      yTranslation.reset();
      Pose2d[] algaePoses = Robot.reefPosesGenerate.getAlgaePoses();

      // Drive to this pose that finds nearest pose from current pose
      algaePose = RobotContainer.driveTrain.getPose().nearest(Arrays.asList(algaePoses));

      if(algaePose != null) {
        Logger.recordOutput("AlgaePose", algaePose);
        xOutput = xTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX()) * Constants.speedMultiTeleop;
        yOutput = yTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getY(), algaePose.getY()) * Constants.speedMultiTeleop;
        // rotOutput = rotation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX()) * Constants.speedMultiTeleop;

        translationVal = xOutput;
        strafeVal = yOutput;
        // rotation = 0;
      }

    Translation2d translation = new Translation2d(translationVal, strafeVal);

    RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotationVal * Constants.maxAngularSpd);
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
 