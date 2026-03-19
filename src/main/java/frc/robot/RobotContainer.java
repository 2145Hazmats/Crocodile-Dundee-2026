// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final PIDController rotationPID = new PIDController(1, 0, 0);

    private boolean P1manualMode = false;
    private boolean P2manualMode = false;

    private final Telemetry logger = new Telemetry(MaxSpeed);
    

    // Replace with CommandPS4Controller or CommandJoystick if needed

    // Unc Controllers
    private final CommandXboxController P1CommandController = new CommandXboxController(ControllerConstants.P1_CONTROLLER_PORT);
    private final CommandXboxController P2CommandController = new CommandXboxController(ControllerConstants.P2_CONTROLLER_PORT);
    private final CommandXboxController m_CommandEverythingController = new CommandXboxController(ControllerConstants.EVERYTHING_CONTROLLER_PORT);
    private final CommandXboxController m_CommandTestingController = new CommandXboxController(ControllerConstants.TESTING_CONTROLLER_PORT);

    // New Controllers
    private final XboxController P1Controller = new XboxController(ControllerConstants.P1_CONTROLLER_PORT);
    private final XboxController P2Controller = new XboxController(ControllerConstants.P2_CONTROLLER_PORT);
    private final XboxController m_EverythingController = new XboxController(ControllerConstants.EVERYTHING_CONTROLLER_PORT);
    private final XboxController m_TestingController = new XboxController(ControllerConstants.TESTING_CONTROLLER_PORT);

    //Subsystems are defined here
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
    private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem(drivetrain);
    private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(drivetrain);
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(drivetrain);

    private final SendableChooser<Command> autoChooser;

     private void registerNamedCommands() {
        NamedCommands.registerCommand("Shoot", autoShootCommand().withTimeout(15));
        NamedCommands.registerCommand("IntakeDOWN", m_IntakeSubsystem.autoIntakeDOWN());
        NamedCommands.registerCommand("IntakeUP", m_IntakeSubsystem.autoIntakeHOME());
        NamedCommands.registerCommand("IntakeUnjam", m_IntakeSubsystem.autoIntakeUnjam());
        NamedCommands.registerCommand("ClimbDown", m_ClimbSubsystem.moveClimbToPosition(ClimbConstants.CLIMB_HOME_POSITION));
        NamedCommands.registerCommand("ClimbUp", m_ClimbSubsystem.moveClimbToPosition(ClimbConstants.CLIMB_UP_POSITION));
    }

    public RobotContainer() {
        
        configureBindings();
        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        m_IntakeSubsystem.resetIntakePosition();

        SmartDashboard.putNumber("Flywheel Setpoint", 0);
        SmartDashboard.putNumber("Hood Setpoint", 12);
        SmartDashboard.putBoolean("P2 Manual Mode", P2manualMode);
    } 

    private void configureBindings() {
    /*-------------------------------------------Driver Controls-------------------------------------------*/  

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1Controller.getRawAxis(1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1Controller.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-P1Controller.getRawAxis(3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Turn turret where we want it
        m_TurretSubsystem.setDefaultCommand(m_TurretSubsystem.turnTurretToAngle(
          () -> drivetrain.getAngleToTarget()));
          /*
          .onlyIf(() -> !manualMode)
        .beforeStarting(Commands.run(() -> m_TurretSubsystem.turnTurretToAngle(
          () -> P2Controller.getRightX() * Math.toRadians(80)), m_TurretSubsystem))
          .onlyIf(() -> manualMode));
        //m_TurretSubsystem.setDefaultCommand());
        */
        /*m_TurretSubsystem.setDefaultCommand(Commands.run(() -> m_TurretSubsystem.setMotor(0), m_TurretSubsystem)); */

        m_SpindexerSubsystem.setDefaultCommand(Commands.run(() -> m_SpindexerSubsystem.SetMotor(0), m_SpindexerSubsystem));
        
        m_ShooterSubsystem.setDefaultCommand(Commands.run(() -> {
          m_ShooterSubsystem.setFlywheelMotor(0.1);
          m_ShooterSubsystem.setFeederMotor(0);
        }, m_ShooterSubsystem));

        //m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.setIntakePosition(IntakeConstants.ACTUATOR_HOME_POSITION));
           
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

    
    
       
  
  
      /*-----------------------------------New Controller Configs----------------------------------- */

      /*-------------------------------P1 Controller-------------------------------- */

      final Trigger P1A = new Trigger(() -> P1Controller.getRawButton(1));
      final Trigger P1B = new Trigger(() -> P1Controller.getRawButton(2));
      final Trigger P1l4 = new Trigger(() -> P1Controller.getRawButton(3));
      final Trigger P1X = new Trigger(() -> P1Controller.getRawButton(4));
      final Trigger P1Y = new Trigger(() -> P1Controller.getRawButton(5));
      final Trigger P1r4 = new Trigger(() -> P1Controller.getRawButton(6));
      final Trigger P1leftBumper = new Trigger(() -> P1Controller.getRawButton(7));
      final Trigger P1rightBumper = new Trigger(() -> P1Controller.getRawButton(8));
      final Trigger P1leftTrigger = new Trigger(() -> P1Controller.getRawButton(9));
      final Trigger P1rightTrigger = new Trigger(() -> P1Controller.getRawButton(10));
      final Trigger P1minus = new Trigger(() -> P1Controller.getRawButton(11));
      final Trigger P1Plus = new Trigger(() -> P1Controller.getRawButton(12));

      /*-------------------------------P2 Controller-------------------------------- */


      final Trigger P2A = new Trigger(() -> P2Controller.getRawButton(1));
      final Trigger P2B = new Trigger(() -> P2Controller.getRawButton(2));
      final Trigger P2l4 = new Trigger(() -> P2Controller.getRawButton(3));
      final Trigger P2X = new Trigger(() -> P2Controller.getRawButton(4));
      final Trigger P2Y = new Trigger(() -> P2Controller.getRawButton(5));
      final Trigger P2r4 = new Trigger(() -> P2Controller.getRawButton(6));
      final Trigger P2rightBumper = new Trigger(() -> P2Controller.getRawButton(8));
      final Trigger P2leftBumper = new Trigger(() -> P2Controller.getRawButton(7));
      final Trigger P2rightTrigger = new Trigger(() -> P2Controller.getRawButton(10));
      final Trigger P2leftTrigger = new Trigger(() -> P2Controller.getRawButton(9));
      final Trigger P2minus = new Trigger(() -> P2Controller.getRawButton(11));
      final Trigger P2Plus = new Trigger(() -> P2Controller.getRawButton(12));

    /*--------------------------------New Controller Commands-------------------------------- */
        
    // slow mode
        P1rightBumper.whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1Controller.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
        .withVelocityY(-P1Controller.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
        .withRotationalRate(-P1Controller.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
      ));

      P1A.whileTrue(drivetrain.applyRequest(() -> brake));
      
      P1B.whileTrue(drivetrain.applyRequest(() -> 
        drive.withVelocityX(-P1Controller.getLeftY() * MaxSpeed)
        .withVelocityY(-P1Controller.getLeftX() * MaxSpeed)
        .withRotationalRate(rotationPID.calculate(drivetrain.getPose2d().getRotation().getRadians(), drivetrain.getAngleToTarget()) * MaxAngularRate)));

      P1Y.and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathfindToPose(PoseConstants.BLUE_SHOOT_POSE));
      P1Y.and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathfindToPose(PoseConstants.RED_SHOOT_POSE));
      

    
    /*-------------------------------------------Driver Controls-------------------------------------------*/  
    


    /*-------------------------------------Operator Controls-------------------------------------*/
      P2Plus.whileTrue(Commands.run(() -> m_SpindexerSubsystem.SetMotor(-1), m_SpindexerSubsystem).finallyDo(
        () -> m_SpindexerSubsystem.SetMotor(0)));

      // Manual Stuffs
      //P2minus.onTrue(Commands.runOnce(() -> P2manualMode = !P2manualMode));
      
      P2rightBumper.whileTrue(
        Commands.run(() -> m_ShooterSubsystem.setFlywheelToSpeed(1750), m_ShooterSubsystem));

      P2rightTrigger.whileTrue(Commands.run(() -> {
        m_SpindexerSubsystem.SetMotor(-0.75);
        m_ShooterSubsystem.setFeederMotor(0.75);
      }));

      P2Y.whileTrue(Commands.run(
        () -> m_ShooterSubsystem.setHoodMotorPosition(ShooterConstants.HOOD_MAX_ANGLE)).alongWith(Commands.run(
        () -> m_ShooterSubsystem.setFlywheelToSpeed(ShooterConstants.FLYWHEEL_PASS_SETPOINT), m_ShooterSubsystem)))
      .whileFalse(Commands.run(
        () -> m_ShooterSubsystem.setHoodMotorPosition(ShooterConstants.HOOD_HOME_ANGLE)));
      

      //Climb controls   
      //P2B.whileTrue(m_ClimbSubsystem.moveClimbToPosition(ClimbConstants.CLIMB_UP_POSITION));
      //P2A.whileTrue(m_ClimbSubsystem.moveClimbToPosition(ClimbConstants.CLIMB_HOME_POSITION));
      
      P2l4.whileTrue(Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(1), m_IntakeSubsystem)
      .finallyDo(() -> m_IntakeSubsystem.setIntakingMotor(0)));
      P2X.whileTrue(Commands.run(() -> m_IntakeSubsystem.setIntakePosition(IntakeConstants.ACTUATOR_ALL_THE_WAY_IN)));

      // Regurgitate the fuel
      P2leftBumper.whileTrue(regurgitateCommand());

      // Intake command -- Puts intake down when pressing down LT, and puts it back up when you let go
      P2leftTrigger
      .whileTrue(
        new ParallelCommandGroup(
          m_IntakeSubsystem.setIntakePosition(IntakeConstants.ACTUATOR_DOWN_POSITION),
          Commands.run(
            () -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.INTAKE_MOTOR_SPEED)
          )
        )
      )
      .whileFalse(
        new ParallelCommandGroup(
          m_IntakeSubsystem.setIntakePosition(IntakeConstants.ACTUATOR_HOME_POSITION),
          Commands.run(
            () -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.INTAKE_MOTOR_SPEED)
          )
          .withTimeout(3)
          .finallyDo(() -> m_IntakeSubsystem.setIntakingMotor(0))
        )
      );

      // Shoot command -- Sets the flywheel speed, waits for it to spin up before starting the spindexer and feeder
      /*P2rightTrigger
       .whileTrue(
        new ParallelCommandGroup(
            Commands.run(() -> { 
              m_ShooterSubsystem.setFlywheelToSpeed(m_ShooterSubsystem.distanceToFlywheelSpeed(drivetrain.getDistanceToTarget()));
              //m_ShooterSubsystem.setHoodMotorPosition(MathConstants.DegreesToRotations(MathUtil.clamp(m_ShooterSubsystem.distanceToHoodAngleDegrees(drivetrain.getDistanceToTarget()), 12, 40)) * ShooterConstants.HOOD_GEAR_RATIO);
            }),
            Commands.waitUntil(() -> m_ShooterSubsystem.isFlywheelNearSetpoint(m_ShooterSubsystem.distanceToFlywheelSpeed(drivetrain.getDistanceToTarget()))).withTimeout(2.5)
                .andThen(Commands.run(() -> {
                    m_ShooterSubsystem.setFeederMotor(0.75);
                    m_SpindexerSubsystem.SetMotor(-0.75);
                    }, m_ShooterSubsystem, m_SpindexerSubsystem)
                )
            )*/

      /*

      P2rightTrigger.whileTrue(Commands.run(
        () -> {
          m_ShooterSubsystem.setFlywheelToSpeed(7000 * P2Controller.getRightTriggerAxis());
          m_ShooterSubsystem.setHoodMotorPosition(12);
        }
        ).onlyIf(() -> manualMode));
        */
      

      // Quick PID stuff for testing
      /*
      P1B.whileTrue(
            Commands.run(
                () -> {
                    m_ShooterSubsystem.setFlywheelToSpeed(SmartDashboard.getNumber("Flywheel Setpoint", 0));
                    m_ShooterSubsystem.setHoodMotorPosition(SmartDashboard.getNumber("Hood Setpoint", 0));
                },
                 m_ShooterSubsystem
            )
      );
      */
  } 

  private Command regurgitateCommand() {
    return Commands.run(
      () -> m_SpindexerSubsystem.SetMotor(1), m_SpindexerSubsystem
      )
      .finallyDo(() -> m_SpindexerSubsystem.SetMotor(0))
      .alongWith(Commands.run(() -> m_ShooterSubsystem.setFeederMotor(-1), m_ShooterSubsystem)
      .finallyDo(() -> m_ShooterSubsystem.setFeederMotor(0)));
  }

  //Command for autonomous control to shoot
  private Command autoShootCommand(){
     return new ParallelCommandGroup(
            Commands.runOnce(() -> m_ShooterSubsystem.setFlywheelToSpeed(ShooterConstants.FLYWHEEL_RPM_SETPOINT)),
            Commands.waitUntil(() -> m_ShooterSubsystem.isFlywheelNearSetpoint(ShooterConstants.FLYWHEEL_RPM_SETPOINT)).withTimeout(2.5)
                .andThen(Commands.run(() -> {
                    m_ShooterSubsystem.setFeederMotor(0.75);
                    m_SpindexerSubsystem.SetMotor(-0.75);
                    }, m_ShooterSubsystem, m_SpindexerSubsystem)
                ).withTimeout(5)
        )
      .finallyDo(() ->
        Commands.runOnce(() -> {
            m_ShooterSubsystem.setFeederMotor(0);
            m_ShooterSubsystem.setFlywheelToSpeed(0);
            m_SpindexerSubsystem.SetMotor(0);
        }, m_ShooterSubsystem, m_SpindexerSubsystem)
      );
    }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();              
  }

  public boolean getP2ManualMode() {
    return P2manualMode;
  }
}
