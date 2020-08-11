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
import frc.robot.Constants.ElevatorConstants;
import io.github.oblarg.oblog.annotations.Log;

public class ElevatorSubsystem extends SubsystemBase {

  private final WPI_VictorSPX elevatorMotor = new WPI_VictorSPX(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID);

  public ElevatorSubsystem() {
    elevatorMotor.configOpenloopRamp(1);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Log
  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(speed);
  }
}