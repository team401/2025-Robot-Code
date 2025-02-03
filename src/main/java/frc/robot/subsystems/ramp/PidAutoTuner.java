package frc.robot.subsystems.ramp;

import java.util.ArrayList;

public class PidAutoTuner {

  private static final double epsilon = 1e-6;
  private ArrayList<Double> tData;
  private ArrayList<Double> outputData;
  private ArrayList<Double> modelData;
  private double Kp, Ki, Kd;
  private double time = 0;

  public PidAutoTuner() {
    tData = new ArrayList<>();
    outputData = new ArrayList<>();
    modelData = new ArrayList<>();
    Kp = 0;
    Ki = 0;
    Kd = 0;
  }

  public void reset() {
    tData = new ArrayList<>();
    outputData = new ArrayList<>();
    modelData = new ArrayList<>();
    Kp = 0;
    Ki = 0;
    Kd = 0;
    time = 0;
  }

  public void addData(double newOutput, double deltaTime) {
    time += deltaTime;
    tData.add(time);
    outputData.add(newOutput);
  }

  public void update() {
    double[] params = estimateParameters(tData, outputData);
    double K_est = params[0];
    double T_est = params[1];
    double theta_est = params[2];

    modelData = new ArrayList<>();
    for (int i = 0; i < tData.size(); i++) {
      modelData.add(fopdtModel(tData.get(i), K_est, T_est, theta_est));
    }

    double[] pidValues = tunePID(K_est, T_est, theta_est);
    Kp = pidValues[0];
    Ki = pidValues[1];
    Kd = pidValues[2];
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

  public double getTime() {
    return time;
  }

  public ArrayList<Double> getTData() {
    return tData;
  }

  public ArrayList<Double> getModelData() {
    return modelData;
  }

  public ArrayList<Double> getOutputData() {
    return outputData;
  }

  public static double fopdtModel(double t, double K, double T, double theta) {
    return K * (1 - Math.exp(-(t - theta) / T)) * (t > theta ? 1 : 0);
  }

  public static double computeMeanSquaredError(
      ArrayList<Double> tData, ArrayList<Double> outputData, double K, double T, double theta) {
    double mse = 0.0;
    for (int i = 0; i < tData.size(); i++) {
      double modelOutput = fopdtModel(tData.get(i), K, T, theta);
      mse += Math.pow(modelOutput - outputData.get(i), 2);
    }
    return mse / tData.size();
  }

  public static double[] estimateParameters(ArrayList<Double> tData, ArrayList<Double> outputData) {
    double K = 1.0, T = 10.0, theta = 5.0; // Initial guess
    double learningRate = 0.01;
    int iterations = 10000;

    for (int iter = 0; iter < iterations; iter++) {
      double mse = computeMeanSquaredError(tData, outputData, K, T, theta);

      double K_grad =
          (computeMeanSquaredError(tData, outputData, K + epsilon, T, theta) - mse) / epsilon;
      double T_grad =
          (computeMeanSquaredError(tData, outputData, K, T + epsilon, theta) - mse) / epsilon;
      double theta_grad =
          (computeMeanSquaredError(tData, outputData, K, T, theta + epsilon) - mse) / epsilon;

      K -= learningRate * K_grad;
      T -= learningRate * T_grad;
      theta -= learningRate * theta_grad;
    }

    return new double[] {K, T, theta};
  }

  public static double[] tunePID(double K, double T, double theta) {
    // Ziegler–Nichols tuning rules for PID
    double Kp = 1.2 * T / (K * theta);
    double Ki = 2 * theta;
    double Kd = 0.5 * theta;
    // double Kp = 0.6 * K;
    // double Ki = 1.2 * K / T;
    // double Kd = 0.075 * K * T;

    return new double[] {Kp, Ki, Kd};
  }
}
