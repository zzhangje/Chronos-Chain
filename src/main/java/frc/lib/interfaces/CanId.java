package frc.lib.interfaces;

public record CanId(int id, String bus) {
  @Override
  public boolean equals(Object o) {
    if (this == o) {
      return true;
    }

    if (!(o instanceof CanId canId)) {
      return false;
    }

    return id == canId.id && bus.equals(canId.bus);
  }
}
