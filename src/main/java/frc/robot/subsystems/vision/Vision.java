package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.CameraIO.Estimate;
import frc.robot.utils.Field;

public class Vision extends SubsystemBase {
  private final CameraIO[] cameraIOs;
  private final CameraIOInputsAutoLogged[] inputs;
  private final VisionConsumer consumer;

  public Vision(VisionConsumer consumer, CameraIO... cameraIOs) {
    this.consumer = consumer;
    this.cameraIOs = cameraIOs;
    this.inputs = new CameraIOInputsAutoLogged[cameraIOs.length];
    for (int i = 0; i < this.inputs.length; i++) {
      this.inputs[i] = new CameraIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameraIOs.length; i++) {
      cameraIOs[i].updateInputs(inputs[i]);
      Logger.processInputs("Camera " + i, inputs[i]);

      if (!inputs[i].connected) continue;

      for (Estimate estimate : inputs[i].estimates) {
        boolean invalidTagCount = estimate.tagCount() == 0;
        boolean invalidHeight = Math.abs(estimate.pose().getZ()) > VisionConstants.MAX_HEIGHT_ERROR.in(Meters);
        boolean invalidX = (estimate.pose().getX() < 0.0) || (estimate.pose().getX() > Field.FIELD_LENGTH.in(Meters));
        boolean invalidY = (estimate.pose().getY() < 0.0) || (estimate.pose().getY() > Field.FIELD_WIDTH.in(Meters));

        if (invalidTagCount || invalidHeight || invalidX || invalidY) continue; // Invalid estimate

        // Calculate standard deviations
        double stdDevFactor = Math.pow(estimate.averageTagDistance(), 2.0) / estimate.tagCount();
        double linearStdDev = VisionConstants.LINEAR_STDDEV_COEFF * stdDevFactor;
        double angularStdDev = VisionConstants.ANGULAR_STDDEV_COEFF * stdDevFactor;

        // Send the estimate over to the pose estimator
        consumer.accept(
          estimate.pose().toPose2d(), 
          estimate.timestamp(), 
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
        );
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
