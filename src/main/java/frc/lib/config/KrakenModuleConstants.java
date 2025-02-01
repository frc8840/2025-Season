package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class KrakenModuleConstants {
    public final int krakenMotorID;
    public final Rotation2d angleOffset;
    
        /**
         * Swerve Module Constants to be used when creating swerve modules.
         *
         * @param krakenMotorID
         * @param angleOffset
         */
        public KrakenModuleConstants(
            int krakenMotorID, Rotation2d angleOffset) {
    
            this.krakenMotorID = krakenMotorID;
            this.angleOffset = angleOffset;
    }
}
