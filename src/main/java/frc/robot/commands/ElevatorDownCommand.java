/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VoltageConstants;
import frc.robot.subsystems.ElevatorSubsystem;

//TODO make it on a timer so it goes down and doesnt wind back up. change to whenpressed

public class ElevatorDownCommand extends CommandBase {

  ElevatorSubsystem elevatorSubsystem;

  public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
  }

  @Override
  public void execute() {

    // System.out.println("voltage elevator -3");
    elevatorSubsystem.setElevatorSpeed(VoltageConstants.ELEVATOR_DOWN_VOLTAGE);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("elevator done");
    elevatorSubsystem.setElevatorSpeed(VoltageConstants.STOP);

    // elevatorSubsystem.setVoltage(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // elevatorSubsystem.isBottomLimitSwitchHit();
  }
}