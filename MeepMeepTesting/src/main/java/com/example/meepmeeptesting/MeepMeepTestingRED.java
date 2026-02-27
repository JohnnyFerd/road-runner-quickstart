package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingRED {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        // Mirrored over X-axis: (x, y, heading) -> (x, -y, -heading)
                        drive.trajectorySequenceBuilder(new Pose2d(-52, -49, Math.toRadians(-127.5)))
                                .strafeTo(new Vector2d(-12, -15)) // x40 while this happening, limelight detection will scan and store the pattern on the obelisk
                                // SHOOT 3 BALLS

                                .turn(Math.toRadians(37.5))           // mirrored turn to pick up new balls
                                // intake on
                                .strafeTo(new Vector2d(-12, -37))
                                .waitSeconds(1)
                                // check if ball registered: if green or purple is detected from spindexer class
                                // if ball is registered, spin the spindexer 1/3
                                .strafeTo(new Vector2d(-12, -43))
                                .waitSeconds(1)
                                // check if ball registered: if green or purple is detected from spindexer class
                                // if ball is registered, spin the spindexer 1/3
                                .strafeTo(new Vector2d(-12, -52.5))
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(-12, -10))
                                // check if ball registered: if green or purple is detected from spindexer class
                                // if ball is registered, spin the spindexer 1/3
                                // balls will have been picked up in PPG order, using the pattern detected by limelight,
                                // move spindexer to that slot, turn on outake to 3000 rpm, lift tongue up, repeat 2 more times

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
