package frc.lib.utils;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;
import lombok.Getter;

public final class TimeBuffer<T> {
  private final double historySize;
  @Getter private final NavigableMap<Double, T> buffer = new TreeMap<>();

  public TimeBuffer(double history_size_sec) {
    historySize = history_size_sec;
  }

  public void add(T sample, double timestamp) {
    buffer.put(timestamp, sample);
    cleanUp();
  }

  public void cleanUp() {
    while (!buffer.isEmpty()) {
      var lastEntryTimestamp = buffer.lastKey();
      var entry = buffer.firstEntry();
      if (lastEntryTimestamp - entry.getKey() >= historySize) {
        buffer.remove(entry.getKey());
      } else {
        return;
      }
    }
  }

  public void clear() {
    buffer.clear();
  }

  public Optional<T> getSample(double timestamp, boolean wantCeil) {
    if (buffer.isEmpty()) {
      return Optional.empty();
    }

    var ceilingEntry = buffer.ceilingEntry(timestamp);
    var floorEntry = buffer.floorEntry(timestamp);

    if (ceilingEntry == null && floorEntry == null) {
      return Optional.empty();
    } else if (ceilingEntry == null) {
      return Optional.of(floorEntry.getValue());
    } else if (floorEntry == null) {
      return Optional.of(ceilingEntry.getValue());
    } else {
      return Optional.of(wantCeil ? ceilingEntry.getValue() : floorEntry.getValue());
    }
  }
}
