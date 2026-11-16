package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Rule;
import java.util.List;
import java.util.Optional;
import org.wpilib.units.measure.Angle;

/**
 * One camera's lens, as Visor's {@code tools/calibrate.py solve} measures it and prints it to paste
 * here: OpenCV's model, at the frame size it was calibrated at, which must be the camera's. A lens
 * belongs to one camera ({@link Camera}): two lenses of one model differ in where their image's
 * center falls, so a camera given another's lens points its poses off.
 *
 * @param width the frame size it was calibrated at
 * @param height the frame size it was calibrated at
 * @param fx focal length across, in pixels
 * @param fy focal length down, in pixels
 * @param cx the optical centre across, in pixels
 * @param cy the optical centre down, in pixels
 * @param distortion OpenCV's coefficients in its order (k1, k2, p1, p2, k3, then k4, k5, k6): 0, 4,
 *     5 or 8 of them
 */
@ConfigRecord
public record Lens(
    int width, int height, double fx, double fy, double cx, double cy, List<Double> distortion) {
  public Lens {
    distortion = List.copyOf(distortion);
    Checks.on(Lens.class)
        .field("width", width, positive())
        .field("height", height, positive())
        .field("fx", fx, positive())
        .field("fy", fy, positive())
        .field("cx", cx, finite())
        .field("cy", cy, finite())
        .field("distortion", distortion, Rule.of(Lens::whyNotOpenCvs))
        .orThrow();
  }

  /** Starts building a lens, one named value at a time. */
  public static LensBuilder.WidthStage builder() {
    return LensBuilder.builder();
  }

  /**
   * The angle it sees across its image's diagonal, corner to corner, as its focal lengths and
   * optical centre put the two corners, leaving out its distortion: the angle between the rays
   * through the image's top left and bottom right corners.
   */
  public Angle diagonalFieldOfView() {
    double[] topLeft = {-cx / fx, -cy / fy, 1};
    double[] bottomRight = {(width - cx) / fx, (height - cy) / fy, 1};
    double dot =
        topLeft[0] * bottomRight[0] + topLeft[1] * bottomRight[1] + topLeft[2] * bottomRight[2];
    double lengths = length(topLeft) * length(bottomRight);
    return Radians.of(StrictMath.acos(dot / lengths));
  }

  private static double length(double[] ray) {
    return StrictMath.sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]);
  }

  private static Optional<String> whyNotOpenCvs(List<Double> distortion) {
    if (!List.of(0, 4, 5, 8).contains(distortion.size())) {
      return Optional.of(
          "has " + distortion.size() + " coefficients; OpenCV's model has 0, 4, 5 or 8");
    }
    if (distortion.stream().anyMatch(c -> !Double.isFinite(c))) {
      return Optional.of("has a coefficient that isn't a number");
    }
    return Optional.empty();
  }
}
