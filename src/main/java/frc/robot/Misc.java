package frc.robot;

import frc.lib.utils.GamePieceTracker;
import frc.reefscape.GamePiece;
import lombok.Getter;

public class Misc {
  @Getter
  private static final GamePieceTracker coral =
      new GamePieceTracker(GamePiece.GamePieceType.CORAL.getName(), 0.5);
}
