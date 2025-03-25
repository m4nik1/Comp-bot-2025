// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

public class TeleopDrive extends Command {
  /** Creates a new TelopDrive. */
  SlewRateLimiter rotationLimiter, translateLimiter, strafeLimiter;

  Pose2d algaePose, coralLeftPose, coralRightPose;

  PIDController xTranslation, yTranslation, rotOutput;

  double targetYaw = 0.0;
  double translationVal, strafeVal, rotationVal;
  boolean targetVisible = true;

  double xOutput, yOutput, rotation;

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
    xTranslation = new PIDController(.03, 0, 0);
    yTranslation = new PIDController(.03, 0, 0);
    // rotation = new PIDController(.5, 0, 0);
    // rotation.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplier = Constants.speedMultiTeleop;
    double getRotation = -RobotContainer.getRightX();


    var results = RobotContainer.photonVision.getUnreadResults();

    if(RobotContainer.getDriverB()) {
      if(!results.isEmpty()) {
        // Gets the latest frame since one has been processed since then
        var latestResult = results.get(results.size() - 1);
        if(latestResult.hasTargets()) { // At least one tag has been seen by the camera
          for (var target : latestResult.getTargets()) {
            int tagId = target.getFiducialId();

            Logger.recordOutput("Found reef tag", Constants.desiredTagIds.contains(tagId));
            // Finds the tags that are associated with the reef
            if(Constants.desiredTagIds.contains(tagId)) {
              double face = Constants.TagToFaceBlue.get(tagId);
              
              // Calculates the face angle in radians
              double thetaCalculate = ((2*Math.PI)*(face/6)+Math.PI) % (2*Math.PI); 

              Translation2d calculatedCoralRight = Robot.reefPosesGenerate.calculateCoralRight(thetaCalculate);

              Logger.recordOutput("Face reef", face);
              coralRightPose = new Pose2d(new Translation2d(Units.inchesToMeters(calculatedCoralRight.getX()), Units.inchesToMeters(calculatedCoralRight.getY())), Rotation2d.fromRadians(thetaCalculate));
            }
          }
        }
      }

      if(coralRightPose != null) {
        xOutput = xTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), coralRightPose.getX()) * Constants.speedMultiTeleop;
        yOutput = yTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getY(), coralRightPose.getY()) * Constants.speedMultiTeleop;
        // rotOutput = rotation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX()) * Constants.speedMultiTeleop;

        translationVal = xOutput;
        strafeVal = yOutput;
        rotationVal = rotationLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getRotation, .01)); // getRotation was negative  
        // rotation = 0;

        Logger.recordOutput("Coral Right Pose", coralRightPose);
      }
    }

    else if(RobotContainer.getDriverX()) {
      if(!results.isEmpty()) {
        // Gets the latest frame since one has been processed since then
        var latestResult = results.get(results.size() - 1);
        if(latestResult.hasTargets()) { // At least one tag has been seen by the camera
          for (var target : latestResult.getTargets()) {
            int tagId = target.getFiducialId();

            Logger.recordOutput("Found reef tag", Constants.desiredTagIds.contains(tagId));
            // Finds the tags that are associated with the reef
            if(Constants.desiredTagIds.contains(tagId)) {
              double face = Constants.TagToFaceBlue.get(tagId);
              
              // Calculates the face angle in radians
              double thetaCalculate = ((2*Math.PI)*(face/6)+Math.PI) % (2*Math.PI); 

              Translation2d calculatedAlgae = Robot.reefPosesGenerate.calculateCoralLeft(thetaCalculate);

              Logger.recordOutput("Face reef", face);
              coralLeftPose = new Pose2d(new Translation2d(Units.inchesToMeters(calculatedAlgae.getX()), Units.inchesToMeters(calculatedAlgae.getY())), Rotation2d.fromRadians(thetaCalculate));
            }
          }
        }
      }

      if(coralLeftPose != null) {
        xOutput = xTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), coralLeftPose.getX()) * Constants.speedMultiTeleop;
        yOutput = yTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getY(), coralLeftPose.getY()) * Constants.speedMultiTeleop;
        // rotOutput = rotation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX()) * Constants.speedMultiTeleop;

        // translationVal = xOutput;
        // strafeVal = yOutput;
        // rotation = 0;

        Logger.recordOutput("Coral Left Pose", coralLeftPose);
      }
    }

    else if(RobotContainer.getDriverY()) {
      if(!results.isEmpty()) {
        // Gets the latest frame since one has been processed since then
        var latestResult = results.get(results.size() - 1);
        if(latestResult.hasTargets()) { // At least one tag has been seen by the camera
          for (var target : latestResult.getTargets()) {
            int tagId = target.getFiducialId();

            Logger.recordOutput("Found reef tag", Constants.desiredTagIds.contains(tagId));
            // Finds the tags that are associated with the reef
            if(Constants.desiredTagIds.contains(tagId)) {
              double face = Constants.TagToFaceBlue.get(tagId);
              
              // Calculates the face angle in radians
              double thetaCalculate = ((2*Math.PI)*(face/6)+Math.PI) % (2*Math.PI); 

              Transform2d calculatedAlgae = Robot.reefPosesGenerate.calculateAlgaePose(thetaCalculate);

              // algaePose = RobotContainer.driveTrain.getRobotPose2d().transformBy(calculatedAlgae);
              Logger.recordOutput("Face reef", face);
              algaePose = new Pose2d(new Translation2d(Units.inchesToMeters(calculatedAlgae.getX()), Units.inchesToMeters(calculatedAlgae.getY())), Rotation2d.fromRadians(thetaCalculate));
            }
          }
        }
      }
      Logger.recordOutput("Found reef tag", false);

      if(algaePose != null) {
        xOutput = xTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX()) * Constants.speedMultiTeleop;
        yOutput = yTranslation.calculate(RobotContainer.driveTrain.getRobotPose2d().getY(), algaePose.getY()) * Constants.speedMultiTeleop;
        // rotOutput = rotation.calculate(RobotContainer.driveTrain.getRobotPose2d().getX(), algaePose.getX()) * Constants.speedMultiTeleop;

        translationVal = xOutput;
        strafeVal = yOutput;
        // rotation = 0;

        Logger.recordOutput("AlgaePose", algaePose);
        Logger.recordOutput("xOutputAlgae", xOutput);
        Logger.recordOutput("yOutputAlgae", yOutput);
        // SmartDashboard.putNumber("xOutput", xOutput);
        // SmartDashboard.putNumber("yOutput", yOutput);
      }

    }
    else {
      speedMultiplier = Constants.speedMultiTeleop;
      double getX = -RobotContainer.getLeftX();
      double getY = -RobotContainer.getLeftY();

      // Remember all these values from the stick are negative
      translationVal = translateLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getY, .01)); // getY was negativeß
      strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getX, .01)); // getX was negative
      rotationVal = rotationLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getRotation, .01)); // getRotation was negative  
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
