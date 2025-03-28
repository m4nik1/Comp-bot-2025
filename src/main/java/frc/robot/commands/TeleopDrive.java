// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TeleopDrive extends Command {
  /** Creates a new TelopDrive. */
  SlewRateLimiter rotationLimiter, translateLimiter, strafeLimiter;

  Pose2d algaePose, coralLeftPose, coralRightPose;

  PIDController xTranslation, yTranslation, rotationPID;

  double targetYaw = 0.0;
  double translationVal, strafeVal, rotationVal;
  boolean targetVisible = true;
  boolean openLoop = true;

  double xOutput, yOutput, rotation;

  int previousTag;

  public TeleopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);

    rotationLimiter = new SlewRateLimiter(3.0);
    translateLimiter = new SlewRateLimiter(3.0);
    strafeLimiter = new SlewRateLimiter(3.0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetYaw = 0.0;
    targetVisible = true;
    xTranslation = new PIDController(.25, 0, 0);
    yTranslation = new PIDController(.35, 0, 0);

    previousTag = -1;
    rotationPID = new PIDController(.5, 0, 0);
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    openLoop = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplier = Constants.speedMultiTeleop;
    double getRotation = -RobotContainer.getRightX();

    if (RobotContainer.getDriverX()) {
      xTranslation.reset();
      yTranslation.reset();
      Pose2d[] leftCoralPoses = Robot.reefPosesGenerate.getCoralRightPositions();

      coralLeftPose = RobotContainer.driveTrain.getPose().nearest(Arrays.asList(leftCoralPoses));

      if (coralLeftPose != null) {
        Logger.recordOutput("Right Coral Pose", coralLeftPose);
        xOutput = xTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), coralLeftPose.getX())
            * Constants.speedMultiTeleop;
        yOutput = yTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getY(), coralLeftPose.getY())
            * Constants.speedMultiTeleop;
        // rotOutput =
        // rotation.calculate(RobotContainer.driveTrain.getRobotPose2d().getRotation().getRadians(),
        // algaePose.getRotation().getRadians()) * Constants.speedMultiTeleop;

        Logger.recordOutput("xOutput", xOutput);
        Logger.recordOutput("yOutput", yOutput);

        // Convert to chassis speeds
        ChassisSpeeds spds = new ChassisSpeeds(xOutput, yOutput, 0);

        RobotContainer.driveTrain.driveRobotRelative(spds);
      }
    } else if (RobotContainer.getDriverB()) {
      xTranslation.reset();
      yTranslation.reset();
      Pose2d[] rightCoralPoses = Robot.reefPosesGenerate.getCoralRightPositions();

      coralRightPose = RobotContainer.driveTrain.getPose().nearest(Arrays.asList(rightCoralPoses));

      if (coralRightPose != null) {
        Logger.recordOutput("Right Coral Pose", coralRightPose);
        xOutput = xTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), coralRightPose.getX())
            * Constants.speedMultiTeleop;
        yOutput = yTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getY(), coralRightPose.getY())
            * Constants.speedMultiTeleop;
        // rotOutput =
        // rotation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(),
        // algaePose.getX()) * Constants.speedMultiTeleop;

        // Convert to chassis speeds
        ChassisSpeeds spds = new ChassisSpeeds(xOutput, yOutput, 0);

        RobotContainer.driveTrain.driveRobotRelative(spds);
      }
    }

    else if (RobotContainer.getDriverY()) {
      xTranslation.reset();
      yTranslation.reset();
      rotationPID.reset();
      
      Pose2d[] algaePoses = Robot.reefPosesGenerate.getAlgaePoses();
      openLoop = false;

      // Drive to this pose that finds nearest pose from current pose
      algaePose = RobotContainer.driveTrain.getPose().nearest(Arrays.asList(algaePoses));

      if (algaePose != null) {
        Logger.recordOutput("AlgaePose", algaePose);
        xOutput = xTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX());
        yOutput = yTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getY(), algaePose.getY());
        // rotation =
        // rotationPID.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(),
        // algaePose.getRotation().getRadians()) * Constants.speedMultiTeleop;

        Logger.recordOutput("xOutput", xOutput);
        Logger.recordOutput("yOutput", yOutput);

        // Convert to chassis speeds
        ChassisSpeeds spds = new ChassisSpeeds(xOutput, yOutput, 0);

        RobotContainer.driveTrain.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(spds, Rotation2d.fromDegrees(0)));
      }

    } else {
      speedMultiplier = Constants.speedMultiTeleop;
      double getX = -RobotContainer.getLeftX();
      double getY = -RobotContainer.getLeftY();
      openLoop = true;

      // Remember all these values from the stick are negative
      translationVal = translateLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getY, .01)); // getY was
                                                                                                        // negativeß
      strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getX, .01)); // getX was negative
      rotationVal = rotationLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getRotation, .01)); // getRotation
                                                                                                           // was
                                                                                                           // negative

      Translation2d translation = new Translation2d(translationVal, strafeVal);

      RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotationVal * Constants.maxAngularSpd);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
