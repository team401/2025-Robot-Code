package frc.robot.constants;

public class MultiPlatformJson {
  static String platform = "none";

  /** Read deploy/constants/Platform.java to see what platform we're on */
  static void initPlatform() throws Exception {
    Platform.synced.loadData();
    switch (Platform.synced.getObject().platform) {
      case "comp":
        break;
      case "testrig":
        break;
      case "hitl":
        break;
      default:
        throw new Exception("Unkown environment " + Platform.synced.getObject().platform);
    }
  }
}
