// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private TalonFX turretMotor; 
  /** Creates a new Turret. */
  public Turret() {
   turretMotor = new TalonFX(Constants.turretConstants.turretMotorId); 
  }

  public void runTurretMotor (double speed) {
    turretMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
