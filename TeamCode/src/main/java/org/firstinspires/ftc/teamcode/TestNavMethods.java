package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Nav Methods", group="Test")
public class TestNavMethods extends LinearOpMode {
    public void runOpMode() {
        Navigation nav = new Navigation(this, telemetry, true);

        waitForStart();

//        nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
//        nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
//        nav.setCollectorExtension(Navigation.CollectorExtension.PARK);
//
//        nav.setLiftHeight(Navigation.LiftHeight.SCORE);
//        nav.setLiftHeight(Navigation.LiftHeight.HOOK);
//        nav.setLiftHeight(Navigation.LiftHeight.LOWER);

//        nav.setCollectionSweeper(Navigation.CollectorSweeper.INTAKE);
//        nav.setCollectorHeight(Navigation.CollectorHeight.COLLECT);
//        nav.setCollectorExtension(Navigation.CollectorExtension.OUT);
//        nav.hold(3f);
//        nav.setCollectorHeight(Navigation.CollectorHeight.HOLD);
//        nav.setCollectorExtension(Navigation.CollectorExtension.DUMP);
//        nav.hold(3f);
//        nav.setCollectorHeight(Navigation.CollectorHeight.DUMP);
//        nav.hold(3f);
//        nav.setCollectionSweeper(Navigation.CollectorSweeper.OFF);

//        nav.setCollectionSweeper(Navigation.CollectorSweeper.INTAKE);
//        nav.setCollectionSweeper(Navigation.CollectorSweeper.OUTTAKE);

        nav.goDistance(20f);
        nav.holdForDrive();
        nav.pointTurnRelative(180f);
        nav.holdForDrive();
        nav.goDistance(20f);
        nav.holdForDrive();
        nav.pointTurnRelative(180f);
        nav.holdForDrive();
    }

}
