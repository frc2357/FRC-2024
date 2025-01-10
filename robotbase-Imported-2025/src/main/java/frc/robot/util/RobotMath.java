package frc.robot.util;

public class RobotMath {
  /**
   * Function to linearly interpolate a value. Will find the two points on with the closest "x"
   * value and do the linear interpolation formula from there
   *
   * <p>Formula: y = y1 + ((x – x1) / (x2 – x1)) * (y2 – y1)
   *
   * @param y2 y2
   * @param y1 y1
   * @param x2 x2
   * @param x1 x1
   * @param x x
   * @param index the index of the row of the 2d curve array
   * @return The unknown value on the curve point (y)
   */
  public static double linearlyInterpolate(double y2, double y1, double x2, double x1, double x) {
    double factor = (x - x1) / (x2 - x1);
    return ((y2 - y1) * factor) + y1;
  }

  /**
   * Finds the index of the lower side of the given x value
   *
   * @param curve The 2D array of values to interpolate on. The first element of the 2nd set of
   *     arrays should be x, while the second element is y Ex. curve[0][0] = known x, curve[0][1] =
   *     unknown y
   * @param x The known value on the current curve point (x)
   * @return The index of the curve array element closest to the given x value that is less than x
   */
  public static int getCurveSegmentIndex(double[][] curve, double x) {
    int segmentIndex = -1;
    for (int i = 0; i < curve.length - 1; i++) {
      if (curve[i][0] > x && curve[i + 1][0] < x) {
        segmentIndex = i;
        break;
      }
    }
    return segmentIndex;
  }
}
