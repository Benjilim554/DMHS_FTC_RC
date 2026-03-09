package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

      /*  myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, 50, Math.toRadians(126)))
                .strafeTo(new Vector2d(-16, 13)) // begin pose
                .strafeToLinearHeading(new Vector2d(-3, 15), Math.toRadians(88))
                .strafeTo(new Vector2d(-3, 49.3))
                .strafeToLinearHeading(new Vector2d(-16, 13), Math.toRadians(126))
                .strafeToLinearHeading(new Vector2d(26, 15), Math.toRadians(87.5))
                .strafeTo(new Vector2d(26, 51.5))
                        .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(-16, 13, Math.toRadians(126)), Math.toRadians(-165)) // travel to shooting zone
                        .strafeToLinearHeading(new Vector2d(52, 8), Math.toRadians(87))
                        .strafeTo(new Vector2d(50, 51.5))
                        .setTangent(Math.toRadians(140))
                        .strafeToLinearHeading(new Vector2d(-16, 13), Math.toRadians(126))
                        .strafeToLinearHeading(new Vector2d(7, 30), Math.toRadians(45))
                .build()); */
        // FAR ZONE VISUALIZER
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -21, Math.toRadians(-180)))
                .strafeToLinearHeading(new Vector2d(52, -13.5), Math.toRadians(-152)) // begin pose

               .strafeToLinearHeading(new Vector2d(38, -16), Math.toRadians(-90))
                        .strafeTo(new Vector2d(38, -52))
                .strafeToLinearHeading(new Vector2d(52, -13.5), Math.toRadians(-152))
                        .strafeToLinearHeading(new Vector2d(43, -60), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(52, -13.5), Math.toRadians(-152))
                .strafeToLinearHeading(new Vector2d(43, -60), Math.toRadians(-180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}