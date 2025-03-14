// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TeleopDrive;
import frc.robot.commands.zeroGyro;
import frc.robot.commands.Auto.CoralDownAuto;
import frc.robot.commands.Auto.ElevatorAuto_HP;
import frc.robot.commands.Auto.ElevatorAuto_l4;
import frc.robot.commands.Auto.RunCoralIntakeAuto;
import frc.robot.commands.CoralPivot.RunPivotManual;
import frc.robot.commands.Elevator_Postions.RunElevatorManual;
import frc.robot.commands.Intakes.AlgaeIn;
import frc.robot.commands.Intakes.CoralIn;
import frc.robot.commands.Intakes.CoralOut;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private SendableChooser<Command> autoChooser;

  // private SendableChooser<Command> autoChooser;

  public static DriveTrain driveTrain;
  public static Elevator elevator;
  public static CoralIntake coralIntake;
  public static CoralPivot coralPivot;
  public static AlgaeIntake algaeIntake;
  // public static Vision photonVision;
  public static Climber climber = new Climber();
  Field2d field;

  static CommandXboxController driver = new CommandXboxController(0);
  static CommandXboxController operator = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    coralPivot = new CoralPivot(); 
    coralIntake = new CoralIntake();
    driveTrain = new DriveTrain();
    elevator = new Elevator();

    driveTrain.setDefaultCommand(new TeleopDrive());
    elevator.setDefaultCommand(new RunElevatorManual());

    field = new Field2d();

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    NamedCommands.registerCommand("Coral_out", new RunCoralIntakeAuto());
    NamedCommands.registerCommand("Pivot_Down", new CoralDownAuto());
    NamedCommands.registerCommand("Elevator_L4", new ElevatorAuto_l4());
    NamedCommands.registerCommand("Elevator_HP", new ElevatorAuto_HP());


    autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    driver.rightBumper().whileTrue(new zeroGyro());
  }

  public static double getLeftYOp() {
    return operator.getLeftY();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static boolean getDriverA() {
    return driver.a().getAsBoolean(); // Add driver A
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

  public static double getLeftX() {
    return driver.getLeftX();
  }
}
