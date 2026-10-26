package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Blue Team One: shoot preloads, collect Garden pollen with Limelight, park by 26 s.
 * Select this on the Driver Hub when our team is Blue Team One.
 */
@Autonomous(name = "Blue Team One Auto", group = "Quantum Storm", preselectTeleOp = "Quantum Storm Two Driver1")
public class BlueTeamOneAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        try (AutoStateMachine auto = new AutoStateMachine(this, AutoStartPosition.BLUE_1)) {
            waitForStart();
            if (!isStopRequested()) auto.run();
        } catch (java.util.concurrent.CancellationException ignored) {
            // STOP during INIT is a normal exit.
        }
    }
}
