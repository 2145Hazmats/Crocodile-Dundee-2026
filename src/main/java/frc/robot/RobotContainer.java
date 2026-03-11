// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

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


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
    private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem(drivetrain);
    private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

    /*-------------------------------------------Driver Controls-------------------------------------------*/  

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-P1Controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1Controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-P1Controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Turn turret where we want it
        m_TurretSubsystem.setDefaultCommand(m_TurretSubsystem.autoTurnTurretCommand());

        // Slow mode
        P1CommandController.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
                drive.withVelocityX(-P1Controller.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1Controller.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-P1Controller.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left))
=======
                drive.withVelocityX(-P1CommandController.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1CommandController.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-P1CommandController.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left))
>>>>>>> Stashed changes
=======
                drive.withVelocityX(-P1CommandController.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1CommandController.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-P1CommandController.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left))
>>>>>>> Stashed changes
=======
                drive.withVelocityX(-P1CommandController.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-P1CommandController.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-P1CommandController.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left))
>>>>>>> Stashed changes
            )
        );

        P1CommandController.y().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(-P1CommandController.getLeftY() * 0.5) // Drive forward with negative Y (forward)
                 .withVelocityY(-P1CommandController.getLeftX() * 0.5) // Drive left with negative X (left)
                 .withRotationalRate(-P1CommandController.getRightX() * 0.5) // Drive counterclockwise with negative X (left)
            )
        );
            
           
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        P1CommandController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        drivetrain.registerTelemetry(logger::telemeterize);

    /*-------------------------------------Robot Controls-------------------------------------*/
    


    /*-------------------------------------Operator Controls-------------------------------------*/
    
    
        //Climb controls   
        P2CommandController.b().whileTrue(m_ClimbSubsystem.climbUp());
        P2CommandController.a().whileTrue(m_ClimbSubsystem.climbDown());
        // m_driverController.x().whileTrue(m_ClimbSubsystem.climbRelease());
    
    
        // Command to run the spindexer at a set speed, stops once you let go of the button y
        P2CommandController.y().whileTrue(Commands.run(
            () -> m_SpindexerSubsystem.SetMotor(1.0), 
            m_SpindexerSubsystem
        )
        .finallyDo(() -> m_SpindexerSubsystem.SetMotor(0)));    
    
        //moves intake to position ready to pick up fuel, and runs intake motor
        //moves back to home position in robot and shuts off motors when releasing left trigger
        P2CommandController.leftTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                m_IntakeSubsystem.setIntakePosition(IntakeConstants.ACTUATOR_DOWN_POSITION),
                Commands.run(
                    () -> m_IntakeSubsystem.setIntakingMotor(0)
                )
            )
        )
        .whileFalse(
            new ParallelCommandGroup(
                m_IntakeSubsystem.setIntakePosition(IntakeConstants.ACTUATOR_HOME_POSITION),
                Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(0))
            )
        );
  
      /*-----------------------------------New Controller Configs----------------------------------- */

      /*-------------------------------P1 Controller-------------------------------- */

      final Trigger P1A = new Trigger(() -> P1Controller.getRawButton(1));
      final Trigger P1B = new Trigger(() -> P1Controller.getRawButton(2));
      final Trigger P1r4 = new Trigger(() -> P1Controller.getRawButton(3));
      final Trigger P1X = new Trigger(() -> P1Controller.getRawButton(4));
      final Trigger P1Y = new Trigger(() -> P1Controller.getRawButton(5));
      final Trigger P1l4 = new Trigger(() -> P1Controller.getRawButton(6));
      final Trigger P1rightBumper = new Trigger(() -> P1Controller.getRawButton(7));
      final Trigger P1leftBumper = new Trigger(() -> P1Controller.getRawButton(8));
      final Trigger P1rightTrigger = new Trigger(() -> P1Controller.getRawButton(9));
      final Trigger P1leftTrigger = new Trigger(() -> P1Controller.getRawButton(10));
      final Trigger P1minus = new Trigger(() -> P1Controller.getRawButton(11));
      final Trigger P1Plus = new Trigger(() -> P1Controller.getRawButton(12));

      /*-------------------------------P2 Controller-------------------------------- */


      final Trigger P2A = new Trigger(() -> P2Controller.getRawButton(1));
      final Trigger P2B = new Trigger(() -> P2Controller.getRawButton(2));
      final Trigger P2r4 = new Trigger(() -> P2Controller.getRawButton(3));
      final Trigger P2X = new Trigger(() -> P2Controller.getRawButton(4));
      final Trigger P2Y = new Trigger(() -> P2Controller.getRawButton(5));
      final Trigger P2l4 = new Trigger(() -> P2Controller.getRawButton(6));
      final Trigger P2rightBumper = new Trigger(() -> P2Controller.getRawButton(7));
      final Trigger P2leftBumper = new Trigger(() -> P2Controller.getRawButton(8));
      final Trigger P2rightTrigger = new Trigger(() -> P2Controller.getRawButton(9));
      final Trigger P2leftTrigger = new Trigger(() -> P2Controller.getRawButton(10));
      final Trigger P2minus = new Trigger(() -> P2Controller.getRawButton(11));
      final Trigger P2Plus = new Trigger(() -> P2Controller.getRawButton(12));

    /*--------------------------------New Controller Commands-------------------------------- */
      // slow mode

      P1rightBumper.whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1Controller.getLeftY() * 0.5) // Drive forward with negative Y (forward)
        .withVelocityY(-P1Controller.getLeftX() * 0.5) // Drive left with negative X (left)
        .withRotationalRate(-P1Controller.getRightX() * 0.5) // Drive counterclockwise with negative X (left)
      ));

      P1A.whileTrue(drivetrain.applyRequest(() -> brake));

    /*-------------------------------------Robot Controls-------------------------------------*/
    


    /*-------------------------------------Operator Controls-------------------------------------*/

      //Climb controls   
      P2B.whileTrue(m_ClimbSubsystem.climbUp());
      P2A.whileTrue(m_ClimbSubsystem.climbDown());
      // P2X.whileTrue(m_ClimbSubsystem.climbRelease());

      // Command to run the spindexer at a full speed, stops once you let go of the button y
      P2Y.whileTrue(Commands.run(
        () -> m_SpindexerSubsystem.SetMotor(1.0), 
        m_SpindexerSubsystem
      )
      .finallyDo(() -> m_SpindexerSubsystem.SetMotor(0)));

      // Regurgitate the fuel
      P2leftBumper.whileTrue(Commands.run(
      () -> m_SpindexerSubsystem.SetMotor(-0.5), m_SpindexerSubsystem
      )
      .finallyDo(() -> m_SpindexerSubsystem.SetMotor(0))
      .alongWith(Commands.run(() -> m_ShooterSubsystem.setFeederMotor(-0.5), m_ShooterSubsystem)
      .finallyDo(() -> m_ShooterSubsystem.setFeederMotor(0))));

      // Intake command -- Puts intake down when pressing down LT, and puts it back up when you let go
      P2leftTrigger
      .whileTrue(
        new ParallelCommandGroup(
          m_IntakeSubsystem.setIntakePosition(IntakeConstants.ACTUATOR_DOWN_POSITION),
          Commands.run(
            () -> m_IntakeSubsystem.setIntakingMotor(0)
          )
        )
      )
      .whileFalse(
        new ParallelCommandGroup(
          m_IntakeSubsystem.setIntakePosition(IntakeConstants.ACTUATOR_HOME_POSITION),
          Commands.run(
            () -> m_IntakeSubsystem.setIntakingMotor(0)
          )
        )
      );
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
      
=======
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

      // Shoot command -- Sets the flywheel speed, waits for it to spin up before starting the spindexer and feeder
      P2rightTrigger
      .whileTrue(
        new ParallelCommandGroup(
            Commands.runOnce(() -> m_ShooterSubsystem.setFlywheelToSpeed(2000), m_ShooterSubsystem),
            Commands.waitUntil(() -> m_ShooterSubsystem.isFlywheelNearSetpoint(2000))
                .andThen(Commands.run(() -> {
                    m_ShooterSubsystem.setFeederMotor(1);
                    m_SpindexerSubsystem.SetMotor(-1);
                    }, m_ShooterSubsystem, m_SpindexerSubsystem)
                )
        )
      )
      .onFalse(
        Commands.runOnce(() -> {
            m_ShooterSubsystem.setFeederMotor(0);
            m_ShooterSubsystem.setFlywheelToSpeed(0);
            m_SpindexerSubsystem.SetMotor(0);
        }, m_ShooterSubsystem, m_SpindexerSubsystem)
      );

      // Quick PID stuff for testing
<<<<<<< Updated upstream
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
      P2CommandController.povLeft().whileTrue(
            Commands.run(
                () -> {
                    m_ShooterSubsystem.setFlywheelToSpeed(SmartDashboard.getNumber("Flywheel Setpoint", 0));
                    m_ShooterSubsystem.setHoodMotorPosition(SmartDashboard.getNumber("Hood Setpoint", 0));
                },
                 m_ShooterSubsystem
            )
      );
  } 

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();              
  }
}
