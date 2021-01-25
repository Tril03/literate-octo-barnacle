// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class moveElevAuto extends CommandBase {
  /** Creates a new moveElevAuto. */
  double encGoalie;

  public moveElevAuto(double encGoal) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.elevator);
    encGoalie = encGoal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.elevator.elevPID.setSetpoint(encGoalie);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.elevator.elevatorMotor.set(ControlMode.PercentOutput,
        Robot.elevator.elevPID.calculate(Robot.elevator.elevatorMotor.getSelectedSensorPosition()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.elevator.elevPID.close();
    Robot.elevator.elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.elevator.elevPID.atSetpoint();
  }
}
