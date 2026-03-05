// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
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
    private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // The robot's subsystems and commands are defined here
    private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
    //private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
    private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

    public RobotContainer() {

    //     m_TurretSubsystem.setDefaultCommand(Commands.run(() -> m_TurretSubsystem.runTurretMotor(operator.getLeftX() * 0.1)
    // , m_TurretSubsystem));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // slow mode

                m_driverController.y().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * 0.5) // Drive counterclockwise with negative X (left)
            ));
            
           
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        m_driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

    
    
    /*-----------------------------------Robot Controls-----------------------------------*/
        
    //Climb controls   
        m_operatorController.b().whileTrue(m_ClimbSubsystem.climbUp());
        m_operatorController.a().whileTrue(m_ClimbSubsystem.climbDown());
   // m_driverController.x().whileTrue(m_ClimbSubsystem.climbRelease());
    
    // Spins turret motor indefinitely :D
    //m_driverController.rightBumper().whileTrue(Commands.run(() -> m_TurretSubsystem.runTurretMotor(0.1))
    //.finallyDo(() -> m_TurretSubsystem.runTurretMotor(0)));
    
    
    // Command to run the spindexer at a set speed, stops once you let go of the button y
    m_operatorController.y().whileTrue(Commands.run(() -> m_SpindexerSubsystem.SetMotor(1.0)
    , m_SpindexerSubsystem)
    .finallyDo(() -> m_SpindexerSubsystem.SetMotor(0)));

    
    // Regurgitate the fuel ;)
    m_operatorController.leftBumper().whileTrue(Commands.run(() -> m_SpindexerSubsystem.SetMotor(-0.5), m_SpindexerSubsystem)
    .finallyDo(() -> m_SpindexerSubsystem.SetMotor(0)));
    m_operatorController.leftBumper().whileTrue(Commands.run(() -> m_ShooterSubsystem.setKickerMotor(-0.5), m_ShooterSubsystem)
    .finallyDo(() -> m_ShooterSubsystem.setKickerMotor(0)));
    
    
    // Intakes the balls and stops when the trigger is let go
    //m_driverController.leftTrigger().whileTrue(Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(0.5), m_IntakeSubsystem)
    //.finallyDo(()->m_IntakeSubsystem.setIntakingMotor(0)));

    
    
    //moves intake to position ready to pick up fuel, and runs intake motor
    //moves back to home position in robot and shuts off motors when releasing left trigger
    m_operatorController.leftTrigger().whileTrue(new ParallelCommandGroup(
      m_IntakeSubsystem.MoveActuatorDown(IntakeConstants.ACTUATOR_DOWN_POSITION),
      Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(0))))
      .whileFalse(new ParallelCommandGroup(
      m_IntakeSubsystem.MoveActuatorHome(IntakeConstants.ACTUATOR_HOME_POSITION),
      Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(0))));
    //puts the flywheel at a setpoint of 2000 RPM, 
    m_operatorController.rightTrigger().whileTrue(Commands.run(()-> m_ShooterSubsystem.flywheelRevUp
    (ShooterConstants.FLYWHEEL_RPM_SETPOINT), m_ShooterSubsystem));
    m_operatorController.rightTrigger().whileTrue(Commands.waitSeconds(1.5).andThen(Commands.run(() -> 
    m_ShooterSubsystem.setKickerMotor(0.5))));

    m_operatorController.povUp().whileTrue(Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(0.35), m_IntakeSubsystem)
    .finallyDo(()-> m_IntakeSubsystem.setIntakingMotor(0)));

    
    
    // Michals moment has come again! ;)
    // Shoots the fuel ;
    /* Joel Check this please, idk if I did the PID right */
    m_driverController.rightTrigger().whileTrue(Commands.run(() -> m_ShooterSubsystem.setKickerMotor(0.5)
    , m_ShooterSubsystem).finallyDo(() -> m_ShooterSubsystem.setKickerMotor(0)));
    m_driverController.rightTrigger().whileTrue(Commands.run(() -> m_ShooterSubsystem.setShooterMotor(0.5)));
  }
    
    

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
