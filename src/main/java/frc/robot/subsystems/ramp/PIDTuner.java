package frc.robot.subsystems.ramp;

import frc.robot.subsystems.ramp.PIDTuner.PIDData;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

// TODO: Maybe make KU_CHANGE_RATE not final
// TODO: make calculation of peak and trough points cached
public class PIDTuner {
  private final int SAMPLES_TO_SAVE = 700;
  private final int MINIMUM_MATCHING = 3;
  private final double MAX_PERCENT_AMPLITUDE_DIFFERENCE = 0.;
  private final double MAX_PERCENT_PERIOD_DIFFERENCE = 0.1;
  private final int PREVENT_DISABLE_TIME = 1;
  private final double KU_CHANGE_RATE = 0.5;

  private double Kp, Ki, Kd, Ku, Tu;
  private double time;
  private ArrayList<PIDData> dataPoints;
  private ArrayList<PIDData> peakPoints;
  private boolean hasStabilized;
  private boolean isStabilized;
  private boolean runningTuning;
  private int preventDisable;
  private ControlType controlType;
  private int peakPointsStartIndex;
  private boolean dirty;

  public enum ControlType {
    P,
    PI,
    PD,
    PID,
    PIR,
    SomeOvershoot,
    NoOvershoot
  }

  public record PID(double P, double I, double D) {}

  public record PIDData(double error, double time) {}

  public PIDTuner(ControlType type) {
    Kp = 0;
    Ki = 0;
    Kd = 0;
    Ku = 0;
    Tu = 0;
    hasStabilized = false;
    isStabilized = false;
    runningTuning = true;
    preventDisable = 0;
    dataPoints = new ArrayList<PIDData>();
    peakPoints = new ArrayList<PIDData>();
    controlType = type;
    peakPointsStartIndex = 0;
    dirty = false;
  }

  public void resumeTuning() {
    runningTuning = true;
    preventDisable = PREVENT_DISABLE_TIME;
  }

  public void recordData(double error, double deltaTime) {
    dirty = true;
    time += deltaTime;
    dataPoints.add(new PIDData(error, time));
    if (dataPoints.size() > SAMPLES_TO_SAVE) {
      PIDData point = dataPoints.remove(0);
      if (peakPoints.size() > 0 && peakPoints.get(0).time == point.time) {
        peakPoints.remove(0);
      }
      peakPointsStartIndex--;
    }
  }

  public void tune() {
    if (runningTuning) {
      if (dirty) {
        calculatePeakPoints();
        ArrayList<Double> periods = getPeriods();
        ArrayList<Double> amplitudes = getAmplitudes();
        isStabilized = isStableOscillation(periods, amplitudes);
      }
      if (isStabilized) {
        hasStabilized = true;
        if (preventDisable > 0) {
          preventDisable--;
        } else {
          runningTuning = false;
          // TODO Maybe put setStablePID here
        }
        setStablePID();
      } else {
        Ku += KU_CHANGE_RATE;
      }
    }
    dirty = false;
    Logger.recordOutput("Ku", Ku);
    Logger.recordOutput("Tu", Tu);
  }

  public void calculatePeakPoints() {
    if (peakPoints.size() > 0) {
      PIDData recalculatePoint = peakPoints.remove(peakPoints.size() - 1);
      peakPointsStartIndex = dataPoints.indexOf(recalculatePoint);
    }
    peakPointsStartIndex = Math.max(peakPointsStartIndex, 0);
    for (int index = peakPointsStartIndex; index < dataPoints.size(); index++) {
      if (index > 0) {
        if (dataPoints.get(index - 1).error > dataPoints.get(index).error) {
          continue;
        }
      }
      if (index < dataPoints.size() - 1) {
        if (dataPoints.get(index + 1).error > dataPoints.get(index).error) {
          continue;
        }
      }
      peakPoints.add(dataPoints.get(index));
    }
    peakPointsStartIndex = dataPoints.size() - 1;
  }

  public ArrayList<Double> getPeriods() {
    ArrayList<Double> periods = new ArrayList<>();
    for (int index = 1; index < peakPoints.size(); index++) {
      periods.add(peakPoints.get(index).time - peakPoints.get(index - 1).time);
    }
    return periods;
  }

  public ArrayList<Double> getAmplitudes() {
    ArrayList<Double> amplitudes = new ArrayList<>();
    for (int index = 1; index < peakPoints.size(); index++) {
      amplitudes.add(peakPoints.get(index).error);
    }
    return amplitudes;
  }

  public boolean isStableOscillation(ArrayList<Double> periods, ArrayList<Double> amplitudes) {
    if (Math.min(periods.size(), amplitudes.size()) < MINIMUM_MATCHING) {
      return false;
    }
    int index = Math.min(periods.size(), amplitudes.size()) - 1;
    double total_period = 0.0;
    double total_amplitude = 0.0;
    for (int offset = 0; offset < MINIMUM_MATCHING; offset++) {
      total_period += periods.get(index - offset);
      total_amplitude += amplitudes.get(index - offset);
    }
    double average_period = total_period / MINIMUM_MATCHING;
    double average_amplitude = total_amplitude / MINIMUM_MATCHING;
    Tu = average_period;
    for (int offset = 0; offset < MINIMUM_MATCHING; offset++) {
      double period_percent_difference =
          Math.abs(periods.get(index - offset) - average_period) / average_period;
      double amplitude_percent_difference =
          Math.abs(amplitudes.get(index - offset) - average_amplitude) / average_amplitude;
      if (period_percent_difference > MAX_PERCENT_PERIOD_DIFFERENCE
          || amplitude_percent_difference > MAX_PERCENT_PERIOD_DIFFERENCE) {
        return false;
      }
    }
    return true;
  }

  private void setStablePID() {
    PID data = getCurrentPID();
    Kp = data.P();
    Ki = data.I();
    Kd = data.D();
  }

  public PID getPID() {
    return new PID(getP(), getI(), getD());
  }

  public double getP() {
    return Kp;
  }

  public double getI() {
    return Ki;
  }

  public double getD() {
    return Kd;
  }

  public double getKu() {
    return Ku;
  }

  public double getTu() {
    return Tu;
  }

  public PID getCurrentPID() {
    return new PID(getCurrentP(), getCurrentI(), getCurrentD());
  }

  private final double SOME_OVERSHOOR_P_CONSTANT = 1.0 / 3.0;

  public double getCurrentP() {
    switch (controlType) {
      case P:
        return 0.5 * Ku;
      case PD:
        return 0.45 * Ku;
      case PI:
        return 0.8 * Ku;
      case PID:
        return 0.6 * Ku;
      case PIR:
        return 0.7 * Ku;
      case SomeOvershoot:
        return SOME_OVERSHOOR_P_CONSTANT * Ku;
      case NoOvershoot:
        return 0.2 * Ku;
    }
    return 0;
  }

  private final double SOME_OVERSHOOR_I_CONSTANT = 2.0 / 3.0;

  public double getCurrentI() {
    switch (controlType) {
      case P:
      case PD:
        return 0;
      case PI:
        return 0.54 * Ku / Tu;
      case PID:
        return 1.2 * Ku / Tu;
      case PIR:
        return 1.75 * Ku / Tu;
      case SomeOvershoot:
        return SOME_OVERSHOOR_I_CONSTANT * Ku / Tu;
      case NoOvershoot:
        return 0.4 * Ku / Tu;
    }
    return 0;
  }

  private final double SOME_OVERSHOOR_D_CONSTANT = 1.0 / 9.0;
  private final double NO_OVERSHOOR_D_CONSTANT = 2.0 / 30.0;

  public double getCurrentD() {
    switch (controlType) {
      case P:
      case PI:
        return 0;
      case PD:
        return 0.10 * Ku * Tu;
      case PID:
        return 0.075 * Ku * Tu;
      case PIR:
      case SomeOvershoot:
        return SOME_OVERSHOOR_D_CONSTANT * Ku * Tu;
      case NoOvershoot:
        return NO_OVERSHOOR_D_CONSTANT * Ku * Tu;
    }
    return 0;
  }

  public PID getTuningPid() {
    return new PID(Ku, 0.0, 0.0);
  }

  public PID getPIDToUse() {
    if (runningTuning) {
      return getTuningPid();
    } else {
      return getPID();
    }
  }

  public boolean hasFinished() {
    return !runningTuning;
  }
}
