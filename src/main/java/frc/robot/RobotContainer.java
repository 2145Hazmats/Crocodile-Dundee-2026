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
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HubSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import pabeles.concurrency.IntRangeTask;

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
    private final XboxController P6Controller = new XboxController(ControllerConstants.P6_CONTROLLER_PORT);

    
    //Subsystems are defined here
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
    private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem(drivetrain);
    private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(drivetrain);
    private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(drivetrain);
    private final HubSubsystem hub = new HubSubsystem(this);
    private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem(drivetrain);

    private final SendableChooser<Command> autoChooser;

     private void registerNamedCommands() {
        NamedCommands.registerCommand("ShootFlywheel", autoFlywheelShootCommand());
        // NamedCommands.registerCommand("StopAllMotors", autoStopAllMotorsCommand());
        NamedCommands.registerCommand("IntakeDown", autoIntakeDOWN());
        NamedCommands.registerCommand("IntakeUp", autoIntakeHOME());
        // NamedCommands.registerCommand("IntakeUnjam", m_IntakeSubsystem.autoIntakeUnjam());
        //NamedCommands.registerCommand("ClimbDown", m_ClimbSubsystem.moveClimbToPosition(ClimbConstants.CLIMB_HOME_POSITION));
        //NamedCommands.registerCommand("ClimbUp", m_ClimbSubsystem.moveClimbToPosition(ClimbConstants.CLIMB_UP_POSITION));
        //NamedCommands.registerCommand("PassAuto", autoPassShootCommand());
        NamedCommands.registerCommand("ShootFeeder", autoSpindexerCommand());
        NamedCommands.registerCommand("StopFeeder", autoFeederStopCommand());
        NamedCommands.registerCommand("IntakeRoller", autoIntakeRoll());
        NamedCommands.registerCommand("IntakeStop", autoIntakeStopCommand());
        NamedCommands.registerCommand("Regurgitate", autoRegurgitateCommand());
        NamedCommands.registerCommand("IntakeMiddle", autoIntakeMiddleCommand());

        // Coast Mode 
        NamedCommands.registerCommand("Coast", Commands.runOnce(() -> drivetrain.configNeutralMode(NeutralModeValue.Coast)));
    }

    public RobotContainer() {
        
        configureBindings();
        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        m_IntakeSubsystem.resetIntakePosition();

        SmartDashboard.putNumber("Flywheel Setpoint", 0);
        SmartDashboard.putNumber("Hood Setpoint", 12);
       
    }

    private void configureBindings() {
    /*-------------------------------------------Default Commands-------------------------------------------*/  

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-P1Controller.getRawAxis(1) * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-P1Controller.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-P1Controller.getRawAxis(3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-P6Controller.getRawAxis(1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-P6Controller.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-P6Controller.getRawAxis(3) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Saturday Sponser Video Default Commands
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-P1Controller.getRawAxis(1) * MaxSpeed* 0.5) // Drive forward with negative Y (forward)
        //             .withVelocityY(-P1Controller.getRawAxis(0) * MaxSpeed * 0.5) // Drive left with negative X (left)
        //             .withRotationalRate(-P1Controller.getRawAxis(3) * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
        //     )

        // );


        // Turn turret where we want it
        m_TurretSubsystem.setDefaultCommand(m_TurretSubsystem.turnTurretToAngleProfiled(
        drivetrain::getAngleToTarget));

        m_FeederSubsystem.setDefaultCommand(Commands.run(() -> m_FeederSubsystem.setFeederMotor(0), m_FeederSubsystem));
        
        m_IntakeSubsystem.setDefaultCommand(Commands.run(()-> m_IntakeSubsystem.setIntakingMotor(0), m_IntakeSubsystem));
          /*
          .onlyIf(() -> !manualMode)
        .beforeStarting(Commands.run(() -> m_TurretSubsystem.turnTurretToAngle(
          () -> P2Controller.getRightX() * Math.toRadians(80)), m_TurretSubsystem))
          .onlyIf(() -> manualMode));
        //m_TurretSubsystem.setDefaultCommand());
        */ 
        /*m_TurretSubsystem.setDefaultCommand(Commands.run(() -> m_TurretSubsystem.setMotor(0), m_TurretSubsystem)); */

        m_SpindexerSubsystem.setDefaultCommand(Commands.run(() -> m_SpindexerSubsystem.SetMotor(0), m_SpindexerSubsystem));
        
        // m_ShooterSubsystem.setDefaultCommand(Commands.run(() -> {
        //   m_ShooterSubsystem.setFlywheelMotor(0.2);
        // }, m_ShooterSubsystem));

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

    /*----------------------------New Controller Commands--------------------*/


    /*----------------------------Saturday Garage Dorr Sponser Video stuffs--*/
      

      final Trigger P6A = new Trigger(() -> P6Controller.getRawButton(1));
      final Trigger P6B = new Trigger(() -> P6Controller.getRawButton(2));
      final Trigger P6l4 = new Trigger(() -> P6Controller.getRawButton(3));
      final Trigger P6X = new Trigger(() -> P6Controller.getRawButton(4));
      final Trigger P6Y = new Trigger(() -> P6Controller.getRawButton(5));
      final Trigger P6r4 = new Trigger(() -> P6Controller.getRawButton(6));
      final Trigger P6leftBumper = new Trigger(() -> P6Controller.getRawButton(7));
      final Trigger P6rightBumper = new Trigger(() -> P6Controller.getRawButton(8));
      final Trigger P6leftTrigger = new Trigger(() -> P6Controller.getRawButton(9));
      final Trigger P6rightTrigger = new Trigger(() -> P6Controller.getRawButton(10));
      final Trigger P6minus = new Trigger(() -> P6Controller.getRawButton(11));
      final Trigger P6Plus = new Trigger(() -> P6Controller.getRawButton(12));

    // Drivin
      // Snail mode        
      P6leftBumper.whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P6Controller.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
        .withVelocityY(-P6Controller.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
        .withRotationalRate(-P6Controller.getRightX() * MaxAngularRate * 0.75) // Drive counterclockwise with negative X (left)
      ));

      // Pre Shootin
      P6rightBumper.whileTrue(
        Commands.run(
          () -> {
            m_ShooterSubsystem.setFlywheelToSpeed(m_ShooterSubsystem.distanceToFlywheelSpeed(drivetrain.getDistanceToTarget()));
            m_FeederSubsystem.setFeederMotor(1);
          }
        )
      )
      .whileFalse(
        Commands.run(
          () -> {
            m_ShooterSubsystem.setFlywheelMotor(0.2);
            m_FeederSubsystem.setFeederMotor(0);
          }
        )
      );

      // Shoots
      P6rightTrigger.whileTrue(Commands.run(
        
        () -> {
          m_SpindexerSubsystem.SetMotor(-1);
        }))
      .whileFalse(Commands.run(
        () -> {
          m_SpindexerSubsystem.SetMotor(0);
        }));


        P6A.whileTrue(m_IntakeSubsystem.setIntakePositionCommand(IntakeConstants.ACTUATOR_MIDDLE_POSITION));
        P6leftBumper.whileTrue(autoRegurgitateCommand());
        // Regurgitate Fuel From Intake
        P6r4.whileTrue(Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.REGURGITATE_SPEED), m_IntakeSubsystem)
        .finallyDo(() -> m_IntakeSubsystem.setIntakingMotor(0)));

    /*----------------------------Driver Controls----------------------------*/  
        
    // slow mode
        P1rightBumper.whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-P1Controller.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
        .withVelocityY(-P1Controller.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
        .withRotationalRate(-P1Controller.getRightX() * MaxAngularRate * 0.75) // Drive counterclockwise with negative X (left)
      ));

      P1A.whileTrue(drivetrain.applyRequest(() -> brake));
      
      P1B.whileTrue(drivetrain.applyRequest(() -> 
        drive.withVelocityX(-P1Controller.getLeftY() * MaxSpeed)
        .withVelocityY(-P1Controller.getLeftX() * MaxSpeed)
        .withRotationalRate(rotationPID.calculate(drivetrain.getPose2d().getRotation().getRadians(), drivetrain.getAngleToTarget()) * MaxAngularRate)));
      
      //When pressed, checks what alliance you are on, and goes to a set pose
      P1Y.and(drivetrain::isAllianceBlue).whileTrue(drivetrain.pathfindToPose(PoseConstants.BLUE_SHOOT_POSE));
      P1Y.and(drivetrain::isAllianceRed).whileTrue(drivetrain.pathfindToPose(PoseConstants.RED_SHOOT_POSE));

      //P1l4.whileTrue(Commands.run(() -> setControllerRumbles(1))).whileFalse(Commands.run(() -> setControllerRumbles(0)));

    /*-------------------------------------Operator Controls-------------------------------------*/

      // Manual Stuffs
      //P2minus.onTrue(Commands.runOnce(() -> P2manualMode = !P2manualMode));
      
      //Shoot Command
      //TODO: Test regression model up to 21 feet, corner is 20.1374095108 feet to the hub
      //(m_ShooterSubsystem.distanceToFlywheelSpeed(drivetrain.getDistanceToTarget())
      //m_ShooterSubsystem.distanceToHoodAngleDegrees(drivetrain.getDistanceToTarget())
      /*
      P2rightBumper.whileTrue(
        Commands.run(
          () -> m_ShooterSubsystem.setFlywheelToSpeed(2750))
        .alongWith(Commands.run(
          () -> m_ShooterSubsystem.setHoodMotorPosition(13.25)
          , m_ShooterSubsystem)));
      */

      // Shoot thingy
      P2rightBumper.whileTrue(
        Commands.run(
          () -> {
            m_ShooterSubsystem.setFlywheelToSpeed(m_ShooterSubsystem.distanceToFlywheelSpeed(drivetrain.getDistanceToTarget()));
            m_FeederSubsystem.setFeederMotor(1);
          }
        )
      )
      .whileFalse(
        Commands.run(
          () -> {
            m_ShooterSubsystem.setFlywheelMotor(0.2);
            m_FeederSubsystem.setFeederMotor(0);
          }
        )
      );
      
      //Sets spindexer and feeder to feed shooter
      P2rightTrigger.whileTrue(Commands.run(
        () -> m_SpindexerSubsystem.SetMotor(-1)))
      .whileFalse(Commands.run(
        () -> {
          m_SpindexerSubsystem.SetMotor(0);
        }));


      
      //Pass function, different flywheel setpoint
      // P2Y.whileTrue(Commands.run(
      //   () -> m_ShooterSubsystem.setFlywheelToSpeed(ShooterConstants.FLYWHEEL_PASS_SETPOINT)
      //   )
      // )
      // .whileFalse(Commands.run(
      //   () -> m_ShooterSubsystem.setHoodMotorPosition(ShooterConstants.HOOD_HOME_ANGLE)));
      

      //Climb controls   
      //P2B.whileTrue(m_ClimbSubsystem.moveClimbToPosition(ClimbConstants.CLIMB_UP_POSITION));
      //P2A.whileTrue(m_ClimbSubsystem.moveClimbToPosition(ClimbConstants.CLIMB_HOME_POSITION));
      
      //Intake on its own, no actuator
      P2l4.whileTrue(Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.INTAKE_MOTOR_SPEED), m_IntakeSubsystem)
      .finallyDo(() -> m_IntakeSubsystem.setIntakingMotor(0)));

      P2B.whileTrue(m_IntakeSubsystem.setIntakePositionCommand(IntakeConstants.ACTUATOR_MIDDLE_POSITION).alongWith(
        Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.INTAKE_MOTOR_SPEED)).withTimeout(0.5)
        .finallyDo(() -> m_IntakeSubsystem.setIntakingMotor(0))
      ));

      // Regurgitate the fuel
      P2leftBumper.whileTrue(autoRegurgitateCommand());

      // Regurgitate Fuel From Intake
      P2r4.whileTrue(Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.REGURGITATE_SPEED), m_IntakeSubsystem)
      .finallyDo(() -> m_IntakeSubsystem.setIntakingMotor(0)));
      

      // Intake command -- Puts intake down when pressing down LT
      P2leftTrigger
      .whileTrue(
        new ParallelCommandGroup(
          m_IntakeSubsystem.setIntakePositionCommand(IntakeConstants.ACTUATOR_DOWN_POSITION),
          Commands.waitUntil(() -> m_IntakeSubsystem.intakeIsNearPosition(IntakeConstants.ACTUATOR_DOWN_POSITION))
                  .andThen(Commands.run(() -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.INTAKE_MOTOR_SPEED)))
        )
      )
      .whileFalse(Commands.run(
            () -> m_IntakeSubsystem.setIntakingMotor(0)
          )
        );

      P2A.whileTrue(m_IntakeSubsystem.setIntakePositionCommand(IntakeConstants.ACTUATOR_HOME_POSITION));

      

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


/*-------------------------------------Auto Commands-------------------------------------*/

  // Auto Pass command for auton
  /*private Command autoPassShootCommand() {
    return Commands.run(
      () -> {   
        m_ShooterSubsystem.setHoodMotorPosition(ShooterConstants.HOOD_MAX_ANGLE);
        m_ShooterSubsystem.setFlywheelToSpeed(ShooterConstants.FLYWHEEL_PASS_SETPOINT);
        m_ShooterSubsystem.setFeederMotor(1.0);
        m_SpindexerSubsystem.SetMotor(-1);
    }  ,m_ShooterSubsystem, m_SpindexerSubsystem
    ).withTimeout(15).finallyDo(
      () -> Commands.runOnce( 
        () -> {
        m_ShooterSubsystem.setHoodMotorPosition(ShooterConstants.HOOD_HOME_ANGLE);
        m_ShooterSubsystem.setFlywheelMotor(0.2);
        m_ShooterSubsystem.setFeederMotor(0);
        m_SpindexerSubsystem.SetMotor(0);
    } , m_ShooterSubsystem, m_SpindexerSubsystem
    ));
  }
    */
  // Stop All Motors command for auton
  // private Command autoStopAllMotorsCommand() {
  //   return Commands.run(() -> {
  //     m_SpindexerSubsystem.SetMotor(0);
  //     m_ShooterSubsystem.setFeederMotor(0);
  //     m_ShooterSubsystem.setFlywheelMotor(0);
  //   }
  //   );
  // }
  //regurgitate command for auton
  private Command autoRegurgitateCommand() {
    return new ParallelCommandGroup(Commands.run(
      () -> m_SpindexerSubsystem.SetMotor(1), m_SpindexerSubsystem), 
      (Commands.run(() -> m_FeederSubsystem.setFeederMotor(-1), m_ShooterSubsystem)));
  }

  private Command autoIntakeMiddleCommand(){
    return m_IntakeSubsystem.setIntakePositionCommand(IntakeConstants.ACTUATOR_MIDDLE_POSITION);
  }

  //Command for autonomous control to shoot
  private Command autoFlywheelShootCommand(){
     return Commands.run(
      () -> { m_ShooterSubsystem.setFlywheelToSpeed(m_ShooterSubsystem.distanceToFlywheelSpeed(drivetrain.getDistanceToTarget()));
      m_FeederSubsystem.setFeederMotor(1);
    }, m_FeederSubsystem, m_ShooterSubsystem);}
  
  private Command autoSpindexerCommand(){
    return Commands.run(()-> 
      m_SpindexerSubsystem.SetMotor(-1)
    , m_SpindexerSubsystem);
  }

   public Command autoIntakeHOME() {
    return m_IntakeSubsystem.setIntakePositionCommand(IntakeConstants.ACTUATOR_HOME_POSITION);
  }

 
  private Command autoFeederStopCommand() {
    return Commands.run(()-> {
      m_FeederSubsystem.setFeederMotor(0);
      m_SpindexerSubsystem.SetMotor(0);
    }, m_FeederSubsystem, m_SpindexerSubsystem);
  }

  public Command autoIntakeDOWN() {
    return m_IntakeSubsystem.setIntakePositionCommand(IntakeConstants.ACTUATOR_DOWN_POSITION);
  }

  public Command autoIntakeRoll(){
    return Commands.runOnce(
      () -> m_IntakeSubsystem.setIntakingMotor(IntakeConstants.INTAKE_MOTOR_SPEED), m_IntakeSubsystem);
  }

  public Command autoIntakeStopCommand(){
    return Commands.runOnce(() -> m_IntakeSubsystem.setIntakingMotor(0), m_IntakeSubsystem);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();              
  }

  public boolean getP2ManualMode() {
    return P2manualMode;
  }

  public void setControllerRumbles(double value) {
    P1Controller.setRumble(RumbleType.kBothRumble, value);
    P2Controller.setRumble(RumbleType.kBothRumble, value);
  }
}
