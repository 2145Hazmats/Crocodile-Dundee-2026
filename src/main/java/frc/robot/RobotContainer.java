// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
  private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  
  private final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    // Default Commands
    m_TurretSubsystem.setDefaultCommand(Commands.run(() -> m_TurretSubsystem.runTurretMotor(m_operatorController.getLeftX() * 0.1), m_TurretSubsystem));

    // Configure the trigger bindings
    configureBindings();

  }
  
  private void configureBindings() {

    // P1 Controls

    // P2 Controls

    // Up and down on the d-pad for moving the climb
    m_driverController.b().whileTrue(m_ClimbSubsystem.climbUp());
    m_driverController.a().whileTrue(m_ClimbSubsystem.climbDown());
    
    // Spins turret motor indefinitely :D
    //m_driverController.rightBumper().whileTrue(Commands.run(() -> m_TurretSubsystem.runTurretMotor(0.1))
    //.finallyDo(() -> m_TurretSubsystem.runTurretMotor(0)));
    
    // command to run the spindexer at a set speed, stops once you let go of the button y
    m_operatorController.rightTrigger().whileTrue(Commands.run(() -> m_SpindexerSubsystem.SetMotor(0.5), m_SpindexerSubsystem)
    .finallyDo(() -> m_SpindexerSubsystem.SetMotor(0)));

    // Regurgitate the balls ;) I like balls.
    m_operatorController.leftBumper().whileTrue(Commands.run(() -> m_SpindexerSubsystem.SetMotor(-0.5), m_SpindexerSubsystem)
    .finallyDo(() -> m_SpindexerSubsystem.SetMotor(0)));
    m_operatorController.leftBumper().whileTrue(Commands.run(() -> m_ShooterSubsystem.setKickerMotor(-0.5), m_ShooterSubsystem)
    .finallyDo(() -> m_ShooterSubsystem.setKickerMotor(0)));
    
    // Michals moment has come again! ;)
    // Shoots the balls ;)
    m_driverController.rightTrigger().whileTrue(Commands.run(() -> m_ShooterSubsystem.setKickerMotor(0.5), m_ShooterSubsystem).finallyDo(() -> m_ShooterSubsystem.setKickerMotor(0)));
    m_driverController.rightTrigger().whileTrue(Commands.run(() -> m_ShooterSubsystem.setShooterMotor(0.5), m_ShooterSubsystem).finallyDo(() -> m_ShooterSubsystem.setShooterMotor(0)));
    
    // Intakes the balls and stops when the trigger is let go
    //m_driverController.leftTrigger().whileTrue(Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(0.5), m_IntakeSubsystem)
    //.finallyDo(()->m_IntakeSubsystem.setIntakingMotor(0)));

    //moves intake to position ready to pick up fuel, and runs intake motor
    //moves back to home position in robot and shuts off motors when releasing left trigger
    m_operatorController.leftTrigger().whileTrue(new ParallelCommandGroup(
      m_IntakeSubsystem.MoveActuatorPosition(IntakeConstants.ACTUATOR_DOWN_POSITION),
      Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.INTAKE_MOTOR_SPEED))))
      .whileFalse(new ParallelCommandGroup(
      m_IntakeSubsystem.MoveActuatorPosition(IntakeConstants.ACTUATOR_HOME_POSITION),
      Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(0))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
