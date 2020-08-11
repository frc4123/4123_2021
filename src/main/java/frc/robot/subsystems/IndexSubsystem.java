/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
// import io.github.oblarg.oblog.annotations.Log;

public class IndexSubsystem extends SubsystemBase {
  /**
   * Creates a new HopperSubsystem.
   */
  private final WPI_VictorSPX indexMotor = new WPI_VictorSPX(HopperConstants.INDEX_MOTOR_CAN_ID);

  public IndexSubsystem() {
    indexMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Set the speed for the index motor
   * 
   * @param speed takes in double between -1.0 and 1.0
   */
  public void indexMotorSpeed(double speed) {
    indexMotor.set(speed);
  }
}