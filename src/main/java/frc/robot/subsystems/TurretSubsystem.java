// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private TalonFX turretMotor;
  private PIDController turretPID;
  private final PositionVoltage m_turretRequest = new PositionVoltage(0).withSlot(0);

  private CommandSwerveDrivetrain m_drivetrain;
  /** Creates a new Turret. */
  public TurretSubsystem(CommandSwerveDrivetrain drivetrain) {
   turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);
   turretPID = new PIDController(TurretConstants.TURRET_P,TurretConstants.TURRET_I, TurretConstants.TURRET_D);
   var slot0Configs = new Slot0Configs();
   slot0Configs.kS = 0;
   slot0Configs.kP = TurretConstants.TURRET_P;
   slot0Configs.kI = TurretConstants.TURRET_I;
   slot0Configs.kD = TurretConstants.TURRET_D;
   turretMotor.getConfigurator().apply(slot0Configs);

   turretMotor.setPosition(MathConstants.DegreesToRotations(TurretConstants.TURRET_STARTING_ANGLE));

   m_drivetrain = drivetrain;
  }

  public void setMotor(double speed) {
    turretMotor.set(speed);
  }
  //Turret Command that uses PID, clamped between setpoint of -90 and 90 degrees
  public Command turnTurretToAngle(DoubleSupplier angleToPointTo) {
    return Commands.run(
      () -> {
        double setpoint = -angleToPointTo.getAsDouble() + m_drivetrain.getPose2d().getRotation().getRadians();
        setMotor(turretPID.calculate(turretMotor.getPosition().getValueAsDouble() / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2, MathUtil.clamp(setpoint, Math.toRadians(-80), Math.toRadians(80))));
      },
      this
    ).finallyDo(
      () -> setMotor(0)
    );
  }

  public Command turnTurretToAngleNew(DoubleSupplier angleToPointTo){
    return Commands.run(
      ()->{
        double setpoint = -angleToPointTo.getAsDouble() + m_drivetrain.getPose2d().getRotation().getRadians();
        turretMotor.setControl(m_turretRequest.withPosition(MathConstants.RadiansToRotations(MathUtil.clamp(setpoint, Math.toRadians(-90), Math.toRadians(90)))));
      });
  }

  public Command autoTurnTurretCommand() {
    var alliance = DriverStation.getAlliance();

    return 

    // Point to Blue Alliance Hub if inside Blue Alliance zone and on Blue Alliance
    turnTurretToAngle(
      () -> m_drivetrain.calculateAngleToFieldPosition(
        PoseConstants.BLUE_ALLIANCE_HUB_LOCATION[0], 
        PoseConstants.BLUE_ALLIANCE_HUB_LOCATION[1]
      )
    ).onlyIf(
      () -> alliance.get() == Alliance.Blue 
      && m_drivetrain.getPose2d().getX() < PoseConstants.BLUE_ALLIANCE_ZONE_X)
    .beforeStarting(

      // Point to Red Alliance Hub if inside Red Alliance zone and on Red Alliance
      turnTurretToAngle(
        () -> m_drivetrain.calculateAngleToFieldPosition(
          PoseConstants.RED_ALLIANCE_HUB_LOCATION[0], 
          PoseConstants.RED_ALLIANCE_HUB_LOCATION[1]
        )
      ).onlyIf(
        () -> alliance.get() == Alliance.Red 
        && m_drivetrain.getPose2d().getX() > PoseConstants.RED_ALLIANCE_ZONE_X)
    .beforeStarting(

      // Point to Blue Alliance right corner if in neutral zone and on right side of the field
      turnTurretToAngle(
        () -> m_drivetrain.calculateAngleToFieldPosition(
          PoseConstants.BLUE_ALLIANCE_RIGHT_CORNER[0],
          PoseConstants.BLUE_ALLIANCE_RIGHT_CORNER[1] 
        )
      ).onlyIf(
        () -> alliance.get() == Alliance.Blue 
        && m_drivetrain.getPose2d().getX() > PoseConstants.BLUE_ALLIANCE_ZONE_X 
        && m_drivetrain.getPose2d().getY() < PoseConstants.CENTER_FIELD_Y
      )
    )
    .beforeStarting(

      // Point to Blue Alliance left corner if in neutral zone and on left side of the field
      turnTurretToAngle(
        () -> m_drivetrain.calculateAngleToFieldPosition(
          PoseConstants.BLUE_ALLIANCE_LEFT_CORNER[0],
          PoseConstants.BLUE_ALLIANCE_LEFT_CORNER[1] 
        )
      ).onlyIf(
        () -> alliance.get() == Alliance.Blue 
        && m_drivetrain.getPose2d().getX() > PoseConstants.BLUE_ALLIANCE_ZONE_X 
        && m_drivetrain.getPose2d().getY() > PoseConstants.CENTER_FIELD_Y
      )
    )
    .beforeStarting(

      // Point to Red Alliance right corner if in neutral zone and on right side of the field
      turnTurretToAngle(
        () -> m_drivetrain.calculateAngleToFieldPosition(
          PoseConstants.RED_ALLIANCE_RIGHT_CORNER[0],
          PoseConstants.RED_ALLIANCE_RIGHT_CORNER[1] 
        )
      ).onlyIf(
        () -> alliance.get() == Alliance.Red 
        && m_drivetrain.getPose2d().getX() < PoseConstants.RED_ALLIANCE_ZONE_X 
        && m_drivetrain.getPose2d().getY() > PoseConstants.CENTER_FIELD_Y
      )
    )
    .beforeStarting(

      // Point to Red Alliance left corner if in neutral zone and on left side of the field
      turnTurretToAngle(
        () -> m_drivetrain.calculateAngleToFieldPosition(
          PoseConstants.RED_ALLIANCE_LEFT_CORNER[0],
          PoseConstants.RED_ALLIANCE_LEFT_CORNER[1] 
        )
      ).onlyIf(
        () -> alliance.get() == Alliance.Red 
        && m_drivetrain.getPose2d().getX() < PoseConstants.RED_ALLIANCE_ZONE_X 
        && m_drivetrain.getPose2d().getY() < PoseConstants.CENTER_FIELD_Y)
    )
    );
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Angle", Units.radiansToDegrees(turretMotor.getPosition().getValueAsDouble()
     / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2));
  }
}
