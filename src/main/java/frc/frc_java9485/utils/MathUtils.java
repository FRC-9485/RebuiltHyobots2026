package frc.frc_java9485.utils;

public class MathUtils {
  public static boolean inRange(double value, double max, double min) {
    return value < max && value > min;
  }

  public static double scope0To360(double value) {
    value %= 360;

    return value;
  }
}
