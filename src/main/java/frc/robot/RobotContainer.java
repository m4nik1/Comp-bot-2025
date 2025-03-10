// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlgaeIn;
import frc.robot.commands.AlgaeOut;
import frc.robot.commands.AngleSet;
import frc.robot.commands.CoralIn;
import frc.robot.commands.CoralOut;
import frc.robot.commands.Elevator_L2;
import frc.robot.commands.RunCoralIntake;
import frc.robot.commands.RunElevatorManual;
import frc.robot.commands.RunPivotManual;
import frc.robot.commands.SetCoralPivot;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // private SendableChooser<Command> autoChooser;

  public static DriveTrain driveTrain;
  public static Elevator elevator;
  public static CoralIntake coralIntake;
  public static CoralPivot coralPivot;
  public static AlgaeIntake algaeIntake;

  static CommandXboxController driver = new CommandXboxController(0);
  static CommandXboxController operator = new CommandXboxController(1);

  private final Field2d field;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    driveTrain = new DriveTrain();
    elevator = new Elevator();
    coralIntake = new CoralIntake();
    coralPivot = new CoralPivot();
    algaeIntake = new AlgaeIntake();

    field = new Field2d();

    // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
    //   field.setRobotPose(pose);
    // });

    // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    //   field.getObject("target pose").setPose(pose);
    // });

    // PathPlannerLogging.setLogActivePathCallback((poses) -> {
    //   field.getObject("path").setPoses(poses);
    // });

    SmartDashboard.putData("Field", field);

    driveTrain.setDefaultCommand(new TeleopDrive());
    // elevator.setDefaultCommand(new RunElevatorManual());
    // coralIntake.setDefaultCommand(new RunCoralIntake())/;
    // coralPivot.setDefaultCommand(new RunPivotManual());

    configureBindings();

    // autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    driver.a().onTrue(new AngleSet());
    // operator.a().onTrue(new SetCoralPivot());
    // operator.b().whileTrue(new Elevator_L2());
    // operator.a().whileTrue(new CoralOut());
    // operator.b().whileTrue(new CoralIn());
    // operator.y().whileTrue(new AlgaeIn());
    // operator.x().whileTrue(new AlgaeOut());
  }

  public static double getLeftYOp() {
    return operator.getLeftY();
  }

  
  public static double getRightYOp() {
    return operator.getRightY();
  }

  public static double getLeftY() {
    return driver.getLeftY();
  }

  public static double getRightX() {
    return driver.getRightX();
  }

  public static boolean getABtn() {
    return operator.a().getAsBoolean();
  }

  public static double getLeftX() {
    return driver.getLeftX();
  }
}
