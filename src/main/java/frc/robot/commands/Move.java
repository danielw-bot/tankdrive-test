/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Move extends CommandBase {
  /**
   * Creates a new Move.
   */

  double time, lSpeed, rSpeed;

  public Move(double time, double lSpeed, double rSpeed) {
    this.time = time;
    this.lSpeed = lSpeed;
    this.rSpeed = rSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.drivetrain.setLeftMotors(lSpeed);
    RobotContainer.drivetrain.setRightMotors(rSpeed);
    setTimeout(time);
    RobotContainer.drivetrain.setLeftMotors(0.0);
    RobotContainer.drivetrain.setRightMotors(0.0);
  }

  private void setTimeout(double time2) {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
