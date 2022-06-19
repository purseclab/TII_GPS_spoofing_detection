# Implementation Note

Please feel free to contact me (kim2956@purdue.edu) if you have any question or suggestion.

## 1. Goal
This PX4 version aims to detect GPS spoofing attacks.

## 2. Hardware and Software Environment

a) **Flight controller and GPS receiver**: Pixhawk 4 Mini and M8N GPS Module with Compass LED Indicator
![image](https://user-images.githubusercontent.com/21173273/174492326-1ffe6e03-6860-4a73-b98b-b5de7a30c2d5.png)

b) **PX4 version**: PX4 v.1.12 (https://github.com/PX4/PX4-Autopilot/commit/2e8918da66af37922ededee1cc2d2efffec4cfb2) <br>

c) **Hardware for GPS spoofing**: <a href="https://greatscottgadgets.com/hackrf/one/" target="_blank">HackRF One</a> which could simulate GPS L1 signal <br> 
![image](https://user-images.githubusercontent.com/21173273/174492672-df488425-1544-4828-859f-82dbbc149279.png)

d) **Software for GPS spoofing**: <a href="https://github.com/osqzss/gps-sdr-sim" target="_blank"> GPS position simulator</a>

## 3. Changed Source Code Files
a) src/drivers/gps/devices/src/ubx.cpp <br>
b) src/drivers/gps/devices/src/ubx.h <br>
c) src/modules/commander/Commander.cpp <br>
d) src/modules/commander/Commander.hpp <br>
e) src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp <br>
f) src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.hpp <br>
g) src/modules/sensors/vehicle_gps_position/params.c <br>

## 4. GPS Spoofing Detection Logic
### Step 1. Initialize configuration parameters related to the GPS noise level
These parameters are used to detect and respond GPS spoofing attacks. <br>
<a href="https://github.com/KimHyungSub/px4_gps/blob/bdd200dde7b05300037f4c53931093f142ba4e54/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp#L158" target="_blank">Link to the code</a>

### Step 2. Measure baseline noise level under the normal condition
Start to measure the baseline noise level after time specified in 'GPS_NOISE_TIME' parameter because the GPS noise level is not stable during the warm or cold booting process in the GPS receiver <br>
<a href="https://github.com/KimHyungSub/px4_gps/blob/bdd200dde7b05300037f4c53931093f142ba4e54/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp#L170" target="_blank">Link to the code</a>
```C++
if ( (gps_noise_measure == true) && (gps_noise_set == false) && (_param_gps_noise_base.get() == 0) ) {
		gps_measure_noise += gps.noise_per_ms;
		gps_measure_noise_cnt++;
	}
```

### Step 3. Store the measured baseline noise level
The measured baseline is stored in 'GPS_NOISE_BASE' parameter. <br>
<a href="https://github.com/KimHyungSub/px4_gps/blob/bdd200dde7b05300037f4c53931093f142ba4e54/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp#L187" target="_blank">Link to the code</a>

```C++
		if ( (gps_noise_set == false) && (gps_noise_measure == true) ){
			gps_noise_set = true;
			int baseline_noise_level = gps_measure_noise/gps_measure_noise_cnt;

			mavlink_log_info(&mavlink_log_pub, "Measured baseline noise level: %d (%d/%d)", baseline_noise_level, gps_measure_noise, gps_measure_noise_cnt);

			_param_gps_noise_base.set(baseline_noise_level);
			_param_gps_noise_base.commit();
		}
```

### Step 4. Check whether there is a GPS spoofing attack every 'GPS_AGC_TIME'
<a href="https://github.com/KimHyungSub/px4_gps/blob/bdd200dde7b05300037f4c53931093f142ba4e54/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp#L198" target="_blank">Link to the code</a>

```C++
if (gps_measure_agc_cnt > _param_gps_agc_time.get()) {

		int agc_avg = gps_measure_agc/gps_measure_agc_cnt;
		_param_gps_agc_avg.set(agc_avg);
		_param_gps_agc_avg.commit();
  ...
```

### Step 5. Check if the current noise level is above the threshold value from the baseline
'GPS_NOISE_THR' parameter denotes the threshold value. Its default value is 20, but you can adjust this threshold. <br>
We conclude that GPS spoofing attack is ongoing under the following conditions: (1) The current noise level is above the threshold value from the baseline and (2) the current AGC is smaller than the moving average of AGC. <br>
**Why?** GPS spoofing attacks make (1) the noise level increase and (2) the AGC decrease. <br>
<a href="https://github.com/KimHyungSub/px4_gps/blob/bdd200dde7b05300037f4c53931093f142ba4e54/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp#L210" target="_blank">Link to the code</a>

```C++
		if ( (gps_noise_set == true) && gps.noise_per_ms > (_param_gps_noise_threshold.get() + _param_gps_noise_base.get()) ) {

			// If the current AGC is smaller than the moving average of AGC, we conclude that GPS spoofing attack is ongoing.
			// Why? GPS spoofing attacks make (1) the noise level increase and (2) the AGC decrease
			if ( gps.automatic_gain_control <= _param_gps_agc_avg.get() ) {
				mavlink_log_info(&mavlink_log_pub, "[DEBUG] Noise_t:%d, Noise_baseline: %d, AGC_t: %d, AGC_avg: %d", gps.noise_per_ms, _param_gps_noise_base.get(), gps.automatic_gain_control, _param_gps_agc_avg.get());
				mavlink_log_info(&mavlink_log_pub, "[WARNING] Detect a GPS spoofing attack");

				// Step 6: Change the 'GPS_SPOOFING' parameter value to trigger a GPS failsafe
				_param_gps_spoofing.set(1);
				_param_gps_spoofing.commit();
			}
		}
```

### Step 6. Respond the detected GPS spoofing attack
We set the barometer sensor as the primary source for height data. <br>
Then, we change the current flight mode into 'LAND' mode as the failsafe behavior. <br>

<a href="https://github.com/KimHyungSub/px4_gps/blob/bdd200dde7b05300037f4c53931093f142ba4e54/src/modules/commander/Commander.cpp#L2115" target="_blank">Link to the code</a>

```C++
		if(_param_gps_spoofing.get() == 1) {
			// Step 1. Set the barometer sensor as the primary source for height data (0: Barometer, 1: GPS, 2: Range sensor, 3: Vision)
			_param_ekf2_hgt_mode.set(0);
			_param_ekf2_hgt_mode.commit();

			// Step 2. Change the current flight mode into 'LAND' mode as the failsafe behavior
			//main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LOITER, _status_flags, _internal_state);
			main_state_transition(_status, commander_state_s::MAIN_STATE_AUTO_LAND, _status_flags, _internal_state);
		}
```

## 5. GPS Spoofing
### Prerequisite
1) You need to have a software-defined radio like <a href="https://greatscottgadgets.com/hackrf/one/" target="_blank">HackRF One</a> which could simulate GPS L1 signal. 

2) Install HackRF package
```bash
sudo apt-get update
sudo apt-get install gnuradio
sudo apt-get install gqrx-sdr
sudo apt-get install hackrf
```

3) Plug in your HackRF

Verify whether it is working or not
```bash
hackrf_info
```

4) Install <a href="https://github.com/osqzss/gps-sdr-sim" target="_blank"> GPS position simulator</a>


### How to execute it?
1) You need to specify the GPS satellite constellation through a GPS broadcast ephemeris file. The daily GPS broadcast ephemeris file (brdc) is a merge of the individual site navigation files into one. The archive for the daily file is:ftp://cddis.gsfc.nasa.gov/gnss/data/daily/

2) Find the latest brdc file for navigation data creation

3) When you use <a href="https://github.com/purseclab/M2MON/tree/main/attacks/GPS_spoofing" target="_blank"> brdc3540.14n</a> as an ephemeris file, you can generate GPS messages by using the command in the below. 
```bash
./gps-sdr-sim -b 8 -e brdc3540.14n -l 31.286502,121.032669,100
```

4) gpssim.bin will be created and the file needs to transferred

5) Spoof the GPS messages 
```bash
hackrf_transfer -t gpssim.bin -f 1575420000 -s 2600000 -a 1 -x 0
```

## 6. Demonstration
### Before conducting a GPS spoofing attack
PX4 measures baseline noise level after finishing a boot process in the GPS receiver. <br>
The measured baseline is 98. <br>
Further, the current flight mode is 'Stabilized'. <br>
![image](https://user-images.githubusercontent.com/21173273/174494290-fee8c8bf-230a-45df-ac63-fee29c1a98d6.png)

### After conducting a GPS spoofing attack
PX4 detects a GPS spoofing attack because (i) the current noise level (119) is above the threshold value (20) from the baseline (98), and (ii) the current AGC value (1872) is smaller than the moving average of AGC (2020).
![image](https://user-images.githubusercontent.com/21173273/174494301-5e385d99-731d-48ff-acd8-dae76aa1803f.png)

## 7. Frequently Asked Question (FAQ)
### Question 1: What features did you use to detect GPS spoofing?
**Answer**: This PX4 version leverages the noise level and AGC without Ephemeris. The reason is that the number of received Ephemeris messages are changed according to the GPS receivers. On the contrary, we notice that the noise level and AGC show stable values regardless of the GPS receivers.

### Question 2: How stable are the default parameter values currently set?
**Answer**: We newly add the following configuration parameters: GPS_NOISE_BASE, GPS_NOISE_TIME, GPS_NOISE_THR, GPS_AGC_AVG, GPS_AGC_TIME, and GPS_SPOOFING. <br>
<a href="https://github.com/KimHyungSub/px4_gps/blob/1cb0cbf9b36897799d26c520070b7eb64c3f8dc3/src/modules/sensors/vehicle_gps_position/params.c#L92" target="_blank">Link to the code</a> <br>
In particular, GPS_NOISE_THR's default value is 20. We believe that this default value is reasonable to distinguish between normal and attack conditions. Yet, we would like to recommand you to test this PX4 version in your environment. 
