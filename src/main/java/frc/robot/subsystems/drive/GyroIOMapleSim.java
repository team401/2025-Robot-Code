package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** IO implementation for Pigeon 2. */
public class GyroIOMapleSim implements GyroIO {
  private final GyroSimulation gyroSimulation;
  private final CorePigeon2 pigeon = new CorePigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id);
  private final Pigeon2SimState pigeonSim = new Pigeon2SimState(pigeon);

  public GyroIOMapleSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
    pigeonSim.setRawYaw(0.0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;

    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
