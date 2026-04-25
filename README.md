An attempt to make a better off-road traction control in BeamNG.Drive.

Version 1.0
Patrick Rich (HammerheadFistpunch)
4/24/26

But aren't there already a bunch of those?  Yes, and I don't like any of them.  The main problem with all TCS mods out there is that they only look at a calculated slip value. The programs don't have a way to compare ideal torque values to requested values.  What that means is that the traction control is very mild in its application to prevent stall.  This custom torquecap lua calculates idea torque values for a given wheel based on engine torque at a given rpm, multiplied by the current gear ratio, multiplied by the transfer case ratio, and finally a fixed ratio for a final drive ratio that is fixed because the math gets funny if you add it in and try to measure it at the same time.  Basically the "crawl ratio" x engine torque x the final drive (4:1 fixed for now). Then divided by 4 for the 4 wheels. Basically canceling it out.

This torque cap ensures that the engine will always have 5% (adjustable) more torque than the brakes can give to any one wheel which prevents stall but applies the max possible clamping force to the wheel.

There are also functions for attack and release profiles, time delays and decay modifiers to prevent pulsing and even an program to look for wheels in the air and max out the torque cap for that wheel.

These all still need work.

Tuning parameters

Intervention hold - time in ms keeps brake pressure high to prevent rollback and surging. hoping to replace this with something more intelligent in the future.
Brake kI - Integral gain. Smooths out slip values over time to ensure clamping stayings consistent and not hard in/out
Brake kP - Proportional gain. how quickly it ramps up pressure.
Max Active Speed - Traction control fade out after this speed (meters per second).
Slip range threshhold - looks at the difference between left and right wheel speeds on the same axle to determine intervention threshold. Eventually I would like to make this more sophisticated and look at expected wheel speed vs reported. 
Cap decay rate - Another way to decay the traction clamp load.
Torque floor - This is the cap
Low speed threshold - This is the speed at the TC is fully active at near zero speeds.


