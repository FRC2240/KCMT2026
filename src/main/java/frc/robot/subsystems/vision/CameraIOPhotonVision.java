package frc.robot.subsystems.vision;

public class CameraIOPhotonVision implements CameraIO{
  public void updateInputs(CameraIOInputs inputs) {
    inputs.connected = true;
    inputs.estimates = new Estimate[0];
  }
}
