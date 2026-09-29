package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class ModuleIOSim implements ModuleIO {
  private SwerveModuleState currentState = new SwerveModuleState(0, Rotation2d.kZero);
  private SwerveModulePosition position = new SwerveModulePosition();
  private double lastTime = 0;

  public void updateInputs(ModuleIOInputs inputs) {
    if (lastTime == 0) lastTime = RobotController.getFPGATime() / 1e6 - 0.02;
    double currentTime = RobotController.getFPGATime() / 1e6;
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // Simulate module steering to target angle (or set directly for simple sim)
    if (DriverStation.isEnabled()) {
      position.angle = currentState.angle;
      position.distanceMeters += currentState.speedMetersPerSecond * dt;
    }

    SwerveModulePosition currentCopy = new SwerveModulePosition(position.distanceMeters, position.angle);
    inputs.position = currentCopy;
    inputs.positionSamples = new SwerveModulePosition[] { currentCopy };
    inputs.timestamps = new double[] { Logger.getTimestamp() };
    inputs.state = new SwerveModuleState(currentState.speedMetersPerSecond, position.angle);
  }

  public void setState(SwerveModuleState state) {
    this.currentState = state;
  }

  public Rotation2d getHeading() {
    return position.angle;
  }
}
