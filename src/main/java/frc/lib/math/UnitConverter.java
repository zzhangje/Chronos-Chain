package frc.lib.math;

import java.util.Objects;
import java.util.function.DoubleUnaryOperator;

/**
 * A generalized unit conversion utility that supports: - Motor systems (rotations, gear ratios) -
 * Sensor systems (distance, angle, temperature) - Custom compound conversions - Bidirectional
 * conversions - Unit system tracking
 */
public final class UnitConverter implements DoubleUnaryOperator {
  private final DoubleUnaryOperator forward;
  private final DoubleUnaryOperator inverse;
  private final String fromUnits;
  private final String toUnits;

  private UnitConverter(
      DoubleUnaryOperator forward, DoubleUnaryOperator inverse, String fromUnits, String toUnits) {
    this.forward = Objects.requireNonNull(forward);
    this.inverse = Objects.requireNonNull(inverse);
    this.fromUnits = fromUnits;
    this.toUnits = toUnits;
  }

  @Override
  public double applyAsDouble(double value) {
    return forward.applyAsDouble(value);
  }

  /** Converts a value in the opposite direction */
  public double convertInverse(double value) {
    return inverse.applyAsDouble(value);
  }

  /** Gets the source units description */
  public String getFromUnits() {
    return fromUnits;
  }

  /** Gets the target units description */
  public String getToUnits() {
    return toUnits;
  }

  // ========== Basic Conversions ==========

  public static UnitConverter identity() {
    return new UnitConverter(x -> x, x -> x, "", "");
  }

  public static UnitConverter scale(double factor) {
    return new UnitConverter(x -> x * factor, x -> x / factor, "", "");
  }

  public static UnitConverter offset(double offset) {
    return new UnitConverter(x -> x + offset, x -> x - offset, "", "");
  }

  // ========== Angle Conversions ==========

  public static UnitConverter radiansToDegrees() {
    return new UnitConverter(Math::toDegrees, Math::toRadians, "rad", "deg");
  }

  public static UnitConverter degreesToRadians() {
    return radiansToDegrees().inverse();
  }

  // ========== Distance Conversions ==========

  public static UnitConverter inchesToMeters() {
    return scale(0.0254).withUnits("in", "m");
  }

  public static UnitConverter metersToInches() {
    return inchesToMeters().inverse();
  }

  public static UnitConverter feetToMeters() {
    return scale(0.3048).withUnits("ft", "m");
  }

  public static UnitConverter metersToFeet() {
    return feetToMeters().inverse();
  }

  // ========== Temperature Conversions ==========

  public static UnitConverter celsiusToFahrenheit() {
    return new UnitConverter(c -> c * 9 / 5 + 32, f -> (f - 32) * 5 / 9, "°C", "°F");
  }

  public static UnitConverter fahrenheitToCelsius() {
    return celsiusToFahrenheit().inverse();
  }

  // ========== Motor/Sensor Specific ==========

  public static UnitEncoderConversion forEncoder(double pulsesPerRevolution) {
    return new UnitEncoderConversion(pulsesPerRevolution);
  }

  public static final class UnitEncoderConversion {
    private final double pulsesPerRev;

    UnitEncoderConversion(double pulsesPerRev) {
      this.pulsesPerRev = pulsesPerRev;
    }

    public UnitConverter pulsesToRotations() {
      return scale(1.0 / pulsesPerRev).withUnits("pulses", "rot");
    }

    public UnitConverter rotationsToPulses() {
      return pulsesToRotations().inverse();
    }

    public UnitConverter pulsesToDistance(double wheelCircumference) {
      return pulsesToRotations().andThen(rotationsToDistance(wheelCircumference));
    }
  }

  public static UnitConverter rotationsToDistance(double circumference) {
    return scale(circumference).withUnits("rot", "m");
  }

  // ========== Composition Methods ==========

  public UnitConverter andThen(UnitConverter after) {
    return new UnitConverter(
        x -> after.forward.applyAsDouble(this.forward.applyAsDouble(x)),
        x -> this.inverse.applyAsDouble(after.inverse.applyAsDouble(x)),
        this.fromUnits,
        after.toUnits);
  }

  public UnitConverter inverse() {
    return new UnitConverter(this.inverse, this.forward, this.toUnits, this.fromUnits);
  }

  public UnitConverter withUnits(String fromUnits, String toUnits) {
    return new UnitConverter(this.forward, this.inverse, fromUnits, toUnits);
  }

  public UnitConverter fromUnits(String fromUnits) {
    return new UnitConverter(this.forward, this.inverse, fromUnits, this.toUnits);
  }

  public UnitConverter toUnits(String toUnits) {
    return new UnitConverter(this.forward, this.inverse, this.fromUnits, toUnits);
  }

  // ========== Helper Methods ==========

  public static UnitConverter linearMap(
      double inputMin, double inputMax, double outputMin, double outputMax) {
    double scale = (outputMax - outputMin) / (inputMax - inputMin);
    return offset(-inputMin).andThen(scale(scale)).andThen(offset(outputMin)).withUnits("", "");
  }
}
