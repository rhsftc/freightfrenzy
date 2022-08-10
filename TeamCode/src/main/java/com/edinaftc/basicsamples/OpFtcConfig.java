package com.edinaftc.basicsamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Config Menu", group = "test")
//@Disabled

public class OpFtcConfig extends OpMode {
//

    FtcConfig ftcConfig = new FtcConfig();

    @Override
    public void init() {
        ftcConfig.init(hardwareMap.appContext, this);
    }

    @Override
    public void init_loop() {
        ftcConfig.init_loop(hardwareMap.appContext, this);
    }

    @Override
    public void loop() {
        telemetry.clearAll();
        // can use configured variables here
        telemetry.addData("ColorIsRed", Boolean.toString(ftcConfig.param.colorIsRed));
        telemetry.addData("DelayInSec", Integer.toString(ftcConfig.param.delayInSec));
        telemetry.addData("AutonType", ftcConfig.param.autonType);
    }

}
