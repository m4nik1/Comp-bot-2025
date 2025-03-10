// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndTarget extends Command {
  /** Creates a new AimAndTarget. */
  boolean targetVisible;
  double targetYaw;
  double turn;
  double vision_kP = 1; // Change this to a constant

  SlewRateLimiter translateLimiter, strafeLimiter;

  public AimAndTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.photonVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translateLimiter = new SlewRateLimiter(1.8);
    strafeLimiter = new SlewRateLimiter(1.8);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplier = Constants.speedMultiTeleop;
    double getX = -RobotContainer.getLeftX();
    double getY = -RobotContainer.getLeftY();
    
    targetYaw = 0.0;
    targetVisible = false;
    turn = 0.0;

    var results = RobotContainer.photonVision.getUnreadResults();

    if(!results.isEmpty()) {
      
      // Gets the latest frame since one has been processed since then
      var result = results.get(results.size() - 1);
      if(result.hasTargets()) {
        for (var target : result.getTargets()) {
          // ID 12 is the HP station
          if(target.getFiducialId() == 12 || target.getFiducialId() == 13 || target.getFiducialId() == 2 || target.getFiducialId() == 1) {
            targetYaw = target.getYaw(); // Gets the targets yaw
            targetVisible = true;
          }
        }
      }
    }

    if(targetVisible) {
      // applies p gain to error to get back a turn value
      turn = -1.0 * targetYaw * vision_kP; 

      SmartDashboard.putNumber("turn vision", turn);
    }

    double translationVal = translateLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getY, .08)); // getY was negativeß
    double strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(getX, .09)); // getX was negative

    Translation2d translationDrive = new Translation2d(translationVal, strafeVal);

    // Set the drivetrain to those speeds
    RobotContainer.driveTrain.drive(translationDrive.times(Constants.maxSpeed), turn * Constants.maxAngularSpd);
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
