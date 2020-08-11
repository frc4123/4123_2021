/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VoltageConstants;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeInCommand extends CommandBase {
  /**
   * Creates a new IntakeCommand.
   */
  private final IntakeSubsystem intakeSubsystem;

  public IntakeInCommand(IntakeSubsystem intakeSubsystem) {
    addRequirements(intakeSubsystem);
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // out of 1
    intakeSubsystem.setIntakeRollerSpeed(VoltageConstants.INTAKE_WHEEL_SPEED_IN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // out of one, stop should be 0
    intakeSubsystem.setIntakeRollerSpeed(VoltageConstants.STOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}