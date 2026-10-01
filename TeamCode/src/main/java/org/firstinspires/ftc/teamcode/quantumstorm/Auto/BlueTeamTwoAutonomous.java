package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous for Blue Team Two.
 * Select this on the Driver Hub when our team is Blue Team Two.
 */
@Autonomous(name = "Blue Team Two Auto", group = "Quantum Storm", preselectTeleOp = "Quantum Storm Two Driver")
public class BlueTeamTwoAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoStateMachine auto = new AutoStateMachine(this, AutoStartPosition.BLUE_2);
        waitForStart();
        auto.run();
    }
}
