package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous for Red Team One.
 * Starting position: red hive facing the robot is tilted up.
 * Select this on the Driver Hub when our team is Red Team One.
 */
@Autonomous(name = "Red Team One Auto", group = "Quantum Storm", preselectTeleOp = "Quantum Storm Two Driver")
public class RedTeamOneAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        AutoStateMachine auto = new AutoStateMachine(this, AutoStartPosition.RED_1);
        waitForStart();
        auto.run();
    }
}
