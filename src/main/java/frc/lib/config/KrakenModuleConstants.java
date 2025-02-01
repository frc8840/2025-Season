package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class KrakenModuleConstants {
    public final int krakenDriveID;
    public final int krakenAngleID;
    public final Rotation2d angleOffset;
    
        /**
         * Swerve Module Constants to be used when creating swerve modules.
         *
         * @param krakenDriveID
         * @param krakenAngleID
         * @param angleOffset
         */
        public KrakenModuleConstants(
            int krakenDriveID, int krakenAngleID, Rotation2d angleOffset) {
    
            this.krakenDriveID = krakenDriveID;
            this.krakenAngleID = krakenAngleID;
            this.angleOffset = angleOffset;
    }
}
