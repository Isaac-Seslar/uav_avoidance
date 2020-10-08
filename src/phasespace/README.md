# PhaseSpace System Documentation

## PhaseSpace Documentation and Software
- Resources:
	- Official documentation and software can be found at: http://phasespace.com/customers/anonymous/
		- user: anonymous
		- pass: guest
	- If you're new to the system, read the following guides as a starting point:
		- These guides are clear, concise and worth reading
		- [Quick Start Guide](http://phasespace.com/customers/anonymous/ImpulseX2E/X2E-Quick-Start-Guide.pdf)
		- [Master Client User Guide](http://phasespace.com/customers/anonymous/ImpulseX2E/X2E-Master-Client-Guide.pdf)
		- [API Guide](http://phasespace.com/customers/anonymous/ImpulseX2E/X2E-API%20Guide.pdf)
	- The Master Client and SDK software can be downloaded here:
		- [Master Client](http://phasespace.com/customers/anonymous/Software/5.2/)
		- [SDK](http://phasespace.com/customers/anonymous/SDK/) (for using the API)

## Primary Apps Used for Setup/Configuration of PhaseSpace System
1. Phasespace Config Manager:
	- Accessed via a web browser at: `192.168.1.230`
		- user: admin
		- pass: phasespace
	- Make sure to disable the proxy in Firefox when accessing this site
1. PhaseSpace Master Client
	- Installed on the Windows PC
		- Log in to Windows PC w/Kerberos or Smart Card
		- Will need SRN connection for first login (disconnect from UAS Lab router first!)
	- Launch `PhaseSpace Master Client` from Windows Start Menu

## Startup Quick Reference
1. Ensure that the Netgear WiFi router is powered on
	- Verify the Phasespace server rack (S1004042) and Windows PC are connected via Ethernet
1. Power on the Phasespace server (black ON/OFF button in center of front panel)
	- Blue LED indicates powered on
	- Green LEDs above Ethernet ports indicate which ports are connected
		- These won't light up until the Master Client is connected
1. Log in to the Windows machine using Kerberos username
1. Open the PhaseSpace Master Client App
	- Select `OWL Configuration` Tab (left side of screen)
	- Make sure the correct address is entered: 192.168.1.230
	- Select the appropriate Profile from the dropdown
		- The profile determines which LED drivers to activate
		- If the desired LED microdriver is not part of the selected profile, the LEDs won't light up
		- See below for instructions on creating profiles
1. Open the PhaseSpace Config Manager in a web browser
	- Use the `System` and `Owl Server` tabs to monitor the system status
	- Use the `LED Devices` tab to manage the LED drivers and profiles

## Setting up a New Profile
1. See "Step 2: Registering Devices" in the [Quick Start Guide](doc/X2E-Quick-Start-Guide.pdf)
	- After this step, a driver or microdriver is paired with the server and ready for configuration
1. Add a profile in the Phasespace Config Manager to configure the LED driver
	- Navigate to http://192.168.0.230 in a web browser
	- Select the `LED Devices` tab
	- Click `Create New Profile`
	- Choose a name, and select the number of LED groups
		- Each LED group is 4 LEDs, so 1 device with 8 LEDs will require at least 2 groups
	- Click `Create this Profile`
	- Add all the devices that will be used simultaneously in this configuration
		- Enter a short, descriptive name for the device
			- ideally, add a label with this name on the LED driver
		- Select the driver type: typically `Microdriver`
		- Select the driver `ID` in the dropdown (driver must be powered on)
		- Click `Continue`
		- Optionally configure the LED strings
			- Give the string a name (e.g. `Upper LEDs`)
			- Select number of LEDs on each string and set the naming/letter of each LED
			- Click `Add this String`
		- Click `Add Driver`
	- Click `Save Profile`
1. Select the profile in the Master Client
	- Select the `OWL Configuration` tab
	- Select `<refresh list>` in the Profile dropdown
	- Selct the desired profile to load it

## Adding a Rigid Body
1. Load the desired profile, and select all the markers to be associated with the rigid body
	- Select by left-clicking and dragging a box around the markers
1. Select `Tools->Rigid Bodies->Create` in the top menu
1. Select the tracker for the rigid body in the `OWL Trackers` tab 
	- Right-click `Modify Tracker` to adjust the translation/orientation
	- Click a marker and then click `Center` to place that marker at the center.

## Calibrating the system
1. Refer to `Step 3: Calibration` in the [Quick Start Guide](doc/X2E-Quick-Start-Guide.pdf) and `Calibration and Alignment` in the [Master Client User's Guide](doc/X2E-Master-Client-Guide.pdf)
1. Make sure the LED driver for the wand has been paired in the PhaseSpace Config Manager and associated with a profile (see [Setting up a New Profile](#setting-up-a-new-profile))
	- A `calibrate` profile has been created for this purpose
1. Select the `calibrate` profile in the PhaseSpace Master Client
1. Connect the wand to its driver (black wire to black wire, blue wire to white-striped wire) and power it on
	- Note that the wand driver is larger than the microdrivers and has a special 2-pin header to connect to the Wand's control board
1. In the top menu of the PhaseSpace Master Client, select `Tools->Calibration`
1. Step 1: Initialize
	- Select `C:\Program Files (x86)\PhaseSpace Master Client\wand.json`
1. Step 2: Capture
	1. Place the wand inside the PhaseSpace volume
		- Check the 2D camera views and make sure the wand's LEDs show up as white dots
		- Ideally, there should be no colored lines going through the points
			- Yellow (too strong/too much light)
			- Brown (too weak/not enough light)
			- Red (camera can't tell which point the light is coming from)
				- Probably from poor alignment of markers relative to that camera
		- Check the `LED Power` box and adjust the value until the colored lines are minimized
			- `35` seems to work well for our setup
			- Just ensure that there are white dots and no colored lines in several camera frames where the wand is roughly centered, since it may not be possible to orient the wand to remove colored lines from all camera frames simultaneously
	1. Click `Capture`
	1. Move the wand through the whole PhaseSpace volume, holding the wand away from body with the LEDs facing outward
		- Continue moving wand around until all camera views are at least 50% filled in.
		- Try to keep wand positioned so it is seen by as many cameras as possible simultaneously
1. Step 3: Calibrate
	- Set the calibration quality with the dropdown on the right
	- `Normal` should be fine for most purposes
	- Click `Calibrate`
	- After four passes, calibration is complete
	- If calibration fails, click `Reset` and repeat the capture process. It is not uncommon to need multiple attempts before successfully calibrating.
1. Click `Save` to preserve the calibration

## Aligning the System
Alignment refers to the creation of an XYZ coordinate system for the motion capture volume.

1. Refer to `Step 4: Alignment` in the [Quick Start Guide](doc/X2E-Quick-Start-Guide.pdf) and `Calibration and Alignment` in the [Master Client User's Guide](doc/X2E-Master-Client-Guide.pdf)
1. If in Calibration mode, click `Align`, otherwise Select `Tools->Alignment` from the top menu
1. Verify the alignment object tracker file is correct: `wand.json` (see Step 6 in Calibration section above)
1. Click `Auto Snapshot` to have the Master Client automatically place alignment points when the wand stops moving.
1. Three points are needed: 1) the origin, 2) a point to define the X axis, and 3) a point to define the Z axis.
	1. Set the origin by placing the wand upright in the center of the space, with the bottom sphere (end furthest from the attached driver) resting on the ground
	1. Set the X axis by moving away from the origin along the direction of X. Place the wand upright, resting on the ground and hold still to record the point.
	1. Set the Z axis in the same way as the X axis.
1. Click `Reset` to start over, or `Save` to preserve the alignment
1. Click `Done` to close the alignment dialog.