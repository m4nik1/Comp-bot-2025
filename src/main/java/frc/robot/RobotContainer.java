// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimAndTarget;
import frc.robot.commands.AngleSet;
import frc.robot.commands.RunCoralIntake;
import frc.robot.commands.RunElevatorManual;
import frc.robot.commands.RunPivotManual;
import frc.robot.commands.CoralPivot90;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Elevator_Postions.Elevator_HP;
import frc.robot.commands.Elevator_Postions.Elevator_L2;
import frc.robot.commands.Elevator_Postions.Elevator_L3;
import frc.robot.commands.Elevator_Postions.Elevator_L4;
import frc.robot.commands.Intakes.AlgaeIn;
import frc.robot.commands.Intakes.AlgaeOut;
import frc.robot.commands.Intakes.CoralIn;
import frc.robot.commands.Intakes.CoralOut;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;

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
  public static Vision photonVision;

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
    photonVision = new Vision();

    field = new Field2d();

    SmartDashboard.putData("Field", field);

    driveTrain.setDefaultCommand(new TeleopDrive());
    elevator.setDefaultCommand(new RunElevatorManual());
    coralPivot.setDefaultCommand(new RunPivotManual());

    configureBindings();

  }

  private void configureBindings() {
    // driver.a().onTrue(new AngleSet());
    // driver.a().onTrue(new AimAndTarget());
    

    // Elevator Positions - Find out what btn should be elevator ground
    operator.b().onTrue(new Elevator_L2());
    operator.a().onTrue(new Elevator_L3());
    operator.y().onTrue(new Elevator_L4());
    operator.x().onTrue(new Elevator_HP());

    // Coral and Algae Intakes
    operator.leftTrigger().onTrue(new AlgaeIn());
    operator.leftBumper().onTrue(new AlgaeOut());

    operator.rightTrigger().onTrue(new CoralIn());
    operator.rightBumper().onTrue(new CoralOut());


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

  public static boolean getStartBtn() {
    return operator.start().getAsBoolean();
  }

  public static double getLeftX() {
    return driver.getLeftX();
  }
}
