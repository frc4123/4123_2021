/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.MiscConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AnglePIDCommand extends PIDCommand {
  /**
   * Creates a new PIDCommandDebug.
   */

  DriveSubsystem driveSubsystem;
  double theParticularChangeInAnglularPositionWeWouldLike;
  static double setterpoint;

  public AnglePIDCommand(DriveSubsystem driveSubsystem) {

    super(new PIDController(1.0, 0.08, .005),
        // This should return the measurement
        () -> driveSubsystem.getGyroAngle(),
        // TThe setpoint
        () -> setterpoint,
        // Output consumer
        output -> {
          output += Math.signum(output) * 2.5;

          driveSubsystem.setOutputVoltage(output, -output);

        }, driveSubsystem);

    // Configure additional PID options by calling `getController` here.
    // if it oscillates uncomment the line bellow
    // getController().setTolerance(AutoAimConstants.ANGLE_TOLERANCE);

    getController().setIntegratorRange(-1.5, 1.5);

    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {

    driveSubsystem.setVoltageCompensation(true, MiscConstants.TURN_VOLTAGE_COMPENSATION_VOLTS);
    // Enable this ^ for auto aim
    // set this class's setterpoint by finding error with the passed in setpoint
    setterpoint = driveSubsystem.getGyroAngle() - theParticularChangeInAnglularPositionWeWouldLike;

    super.initialize();

  }

  // method for passing in setpoint
  public void setThePerticularChangeInAnglularPositionWeWouldLike(
      double thePerticularChangeInAnglularPositionWeWouldLike) {
    this.theParticularChangeInAnglularPositionWeWouldLike = thePerticularChangeInAnglularPositionWeWouldLike;
  }

  @Override
  public void execute() {
    // System.out.println("turntoangle_position_error " +
    // getController().getPositionError());
    super.execute();
  }

  @Override
  public void schedule(boolean interruptible) {
    super.schedule(interruptible);
    // System.out.println("Schedule");
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    driveSubsystem.setVoltageCompensation(false, MiscConstants.TURN_VOLTAGE_COMPENSATION_VOLTS);
  }

  @Override
  public boolean isFinished() {

    return getController().atSetpoint();
  }
}