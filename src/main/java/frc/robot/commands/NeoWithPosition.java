package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoMotor;

public class NeoWithPosition extends CommandBase {
  private final NeoMotor mNeoMotor = NeoMotor.getInstance(); 
  private final double desiredPosition; 
  public NeoWithPosition (double desiredPosition) { 
    this.desiredPosition = desiredPosition; 
    addRequirements(mNeoMotor);
  }

  @Override
  public void initialize() {
    mNeoMotor.setPositionOutput(desiredPosition); 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(desiredPosition - mNeoMotor.getRelativeEncoderPosition()) <= 0.02;
  }
}
