package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

/** IO implementation for Pigeon 2. */
public class GyroIOMapleSim implements GyroIO {
  private final GyroSimulation gyroSimulation;
  private final Pigeon2SimState pigeon = new Pigeon2SimState(
    new CorePigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id));

  public GyroIOMapleSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
    pigeon.setRawYaw(0.0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

  }
}
