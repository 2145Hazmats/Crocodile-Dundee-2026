// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX turretMotor;
  private PIDController turretPID;

  private CommandSwerveDrivetrain m_drivetrain;
  /** Creates a new Turret. */
  public TurretSubsystem(CommandSwerveDrivetrain drivetrain) {
   turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_ID);
   turretPID = new PIDController(TurretConstants.TURRET_P,TurretConstants.TURRET_I, TurretConstants.TURRET_D);

   turretMotor.setPosition(0);

   m_drivetrain = drivetrain;
  }

  public void setMotor(double speed) {
    turretMotor.set(speed);
  }

  public Command turnTurretToHub() {
    return Commands.run(
      () -> {
        double angle = m_drivetrain.calculateAngleToHub();
        setMotor(turretPID.calculate(turretMotor.getPosition().getValueAsDouble() / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2, -angle + m_drivetrain.getPose2d().getRotation().getRadians()));
      },
      this
      ).finallyDo(
      () -> setMotor(0)
    );
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Angle", Units.radiansToDegrees(turretMotor.getPosition().getValueAsDouble() / TurretConstants.TURRET_GEAR_RATIO * Math.PI * 2));
  }
}
