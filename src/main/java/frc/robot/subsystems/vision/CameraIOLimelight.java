package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.LimelightHelpers;

public class CameraIOLimelight implements CameraIO {
  private String name;
  private Supplier<Rotation2d> rotationSupplier;

  private final DoubleArraySubscriber megaTag2Subscriber;
  private final DoubleSubscriber latencySubscriber;

  public CameraIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.name = name;
    this.rotationSupplier = rotationSupplier;

    latencySubscriber = NetworkTableInstance.getDefault()
                        .getDoubleTopic("tl")
                        .subscribe(0.0);
    megaTag2Subscriber = NetworkTableInstance.getDefault()
                         .getTable(name)
                         .getDoubleArrayTopic("botpose_orb_wpiblue")
                         .subscribe(new double[0], PubSubOption.sendAll(true));
  }

  public void updateInputs(CameraIOInputs inputs) {
    // If the camera has not published in a while, assume it is disconnected
    inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250; 

    LimelightHelpers.SetRobotOrientation(name, rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

    TimestampedDoubleArray[] queue = megaTag2Subscriber.readQueue();
    List<Estimate> estimates = new LinkedList<>();

    for (TimestampedDoubleArray entry : queue) {
      double[] rawData = entry.value;
      if (rawData.length < 11) continue; // Malformed

      Pose3d pose = LimelightHelpers.toPose3D(rawData);
      double latency = rawData[6] / 1000.0;
      double timestamp = (entry.timestamp / 1e6) - latency;
      double averageTagDistance = rawData[9];
      int tagCount = (int) rawData[7];

      estimates.add(new Estimate(pose, timestamp, tagCount, averageTagDistance));
    }

    inputs.estimates = estimates.toArray(new Estimate[estimates.size()]);
  }
}
