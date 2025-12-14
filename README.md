
This repository is a fork of this "mbrusa" project https://github.com/emmebrusa/TSDZ2-Smart-EBike-1 that was developed for TSDZ2 Tongsheng motor

The purpose here is to have a version that can be used on a TSDZ8 Tongsheng motor.

Changes were required because TSDZ8 uses a different microprocessor (XMC1302).

This version is supposed to provide the same functionalities and to support the same (4 or more) displays as the mbrusa project above.

Compare to the TSDZ8 original firmware there are some expected benefits:
* The user can adapt many parameters to his preferences.
* The display can show more data (without having to change the display firmware) and allow you to make a few changes to the setup.

As the OSF TSDZ8 version is similar to TSDZ2 version, it is good to look at the TSDZ2 sites: 
* look at Endless Sphere forum reference thread: [endless-sphere.com.](https://endless-sphere.com/forums/viewtopic.php?f=30&t=110682).
* see the [wiki](https://github.com/emmebrusa/TSDZ2-Smart-EBike-1/wiki) from mbrusa

Still this version differs with :
* it requires a Jlink device instead of a stlink device to flash the firmware and the configuration in the controller
* it does not require to install additionnal software in order to compile the firmware yourself. The firmware is already precompiled on this site. Still if you want to change some code, see below the tools to be installed. 
* it is possible to fine tune one parameter (global offset angle in range -5/+5) in order to take care of litle differences (tolerances) that could exist between different motors. This is optionnal.


# IMPORTANT : at this stage, this is just a beta version. There are probably some bugs. Try it at your own risk!!!!
* Installing this firmware will void your warranty of the TSDZ8 mid drive.
* We are not responsible for any personal injuries or accidents caused by use of this firmware.
* There is no guarantee of safety using this firmware, please use it at your own risk.
* We advise you to consult the laws of your country and tailor the motor configuration accordingly.
* Please be aware of your surroundings and maintain a safe riding style.

[![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/donate/?hosted_button_id=AD6HTLE53YDXW)

To use this firmware, you will have to:

* Donwload this firmware
* Use a Segger Jlink device and a cable (this device replace the Stlink used for TSDZ2)
* Flash the compiled firmware on the TSDZ8 controller
* Generate a file with your preferences (using the GUI configurator that is common for TSDZ2 and TSDZ8)
* Flash the hex file that has been generated with the configurator.
* Fine tune some parameters (optional) + monitor (optional) 

If you have questions on this Tsdz8 project, you can ask on this forum:
https://endless-sphere.com/sphere/threads/new-tsdz8-pswpower.120798/page-12

If you find bugs, best is to open an issue in github : https://github.com/mstrens/OSF


# 1.Download this firmware

Download or clone this repository. 
If you downloaded, unzip the archive where you want.

# 2.Preparing Jlink

You need a Segger Jlink device and a cable.
You can get it from here : https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit

This link provides also a link to an archive with all the files for flashing. You can download and unzip it.

Note: I tested with a chinese jlink clone V9 from aliexpress and it worked too.

For the cable, you can also make your own cable with a speed sensor extension cord for TSDZ2 like this:https://fr.aliexpress.com/item/1005007479961045.html?spm=a2g0o.order_list.order_list_main.120.21ef5e5bFWfkqS&gatewayAdapt=glo2fra

I cut the extension cable and connect it directly to the Jlink flat cable based on:
- the diagram of Jlink connector : https://www.segger.com/products/debug-probes/j-link/technology/interface-description/
- the picture of speed cable connector : https://empoweredpeople.co.uk/2020/05/28/tongsheng-tsdz2-what-firmware-options-are-there/
In my case (be careful the colors may be different)
- red = SWDIO = TMS = pin 7 of Jlink
- black = SWCLK = TCK = pin 9 of Jlink
- brown = Vcc = VTRef = pin 1 of Jlink
- orange = Grnd = e.g. pin 4 of Jlink

It can also be useful to look at this link:  https://www.facebook.com/photo?fbid=7877850202251083&set=pcb.430249463263185.
It is safe to check that the extension cable you get uses the same colors for the same pins on the connector because some cables could use different colors/pin out.

Tip: After cutting the extension cord in 2 parts, I used the one with the female connector to connect to the motor (and to Jlink).
I also reconnected the wires from the part with the male connector. The advantage is that I can also connect the male connector to the speed sensor.  This avoid having to connect/disconnect the cable each time you flash the controller during the setup phase.


IMPORTANT POINT : 
If the controller is powered by the battery AND simultaneously by the Jlink device, there is a huge risk that it would be DAMAGED and unusable. Torque sensor driver could be destroyed. You would have to replace a mosfet transistor named Q7 on the controller board. Sill replacing it is not easy and require some skill.


By default Chinese clone Jlink (as the one provided by ebikestuff) usualy provides 3.3V on the Vref pin.
So take care to never power the motor with the battery when Jlink is connected.

For safety, I would recommend to avoid that Jlink provides the 3.3V. This can be achieved in 2 ways:
- if you make your own cable, just do not connect Vref from jlink to the cable
- if you use a cable where Vref is connected, you can (as far I tested) open the case of chinese Jlink device and remove a jumper. Then check with a voltmeter that the device does not provide anymore the 3.3V

Note : original Segger jlink devices does not provide 3.3V on Vref pin by default. Still it can provide the 3.3V if you send some command (see Segger doc if you want to enable it).


# 3.Flash the firmware

To flash, you can follow the instructions from here to know how to use Jlink: 
https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit


The HEX file to upload is the one you downloaded from this github site at step 1.
It is in folder "files_to_flash" and named OSF_TSDZ8_Vxx_xx.hex where xx.xx is a version number.

IMPORTANT NOTE: as said before, for flashing, the motor should not be powered by the battery AND simulterneously by the jlink device. When powered by the battery AND Jlink device, there is a huge risk to damage the controller.

So you must OR disconnect the battery (or at least power it OFF) OR take care that your Jkink does not provide power on Vref pin to the controller.


Take also care to use the menu "manual programming" in J-flash and not the menu "Production programming F7".
The menu "Production programming" has a default setup that erases totally the chip before flashing.
This can be an issue because OSF requires flashing 2 different hex files (the OSF firmware and the configuration hex file).
While flashing one, you usually do not want to erase the other part of the flash memory.
Note : J-flash allows to disable the erase step when using "production programming" but this requires a specific additional action. 

# 4.Generate the configuration file

The configuration file is generated with the same tool as TSDZ2: JavaConfigurator.jar.
A copy of this tool is provided in this repository. You have to use this copy because there can be some different parameters depending on the versions.


To know how to use the JavaConfigurator and the purpose of all parameters, see the TSDZ2 manuals at https://github.com/emmebrusa/TSDZ2-Smart-EBike-1/tree/master/manuals

Take care that there are still a few differences/points of attention about Torque sensor
* calibrated MUST be enabled because you have to fill Pedal torque ADC offset (no weight) and  Pedal torque ADC max (max weight)


* Pedal torque ADC step is important because it is used to convert raw values provided by the torque sensor into the human torque and so also to calculate the human power and the requested motor power. Measures done on only one TSDZ8 motor shows that the raw values provided by the torque sensor are proportional to the weight on the pedal up to about 50Kg (above there is a saturation) and that the raw value at 50kg seems to be about 85 % of the total range (difference between max and offset). In TSDZ8 firmware, ADC values are normalised in order to have a range of 160 steps. So Pedal torque ADC step could be estimated with : 50kg *167 / (85% * 160) = 61. Note: it could be that this value is different for your motor.

* Pedal torque ADC step must be correct in order to get a correct value of the human power (see mbrusa instructions). OSF TSDZ8 uses this field even when calibrated is enabled (on the opposite to TSDZ2)


* Pedal torque ADC step adv is not used (even if calibrated is disabled)
* Pedal torque ADC offset adjustement is not used.
* Pedal torque ADC range adj is used to increase/decrease sensitivity for low pressure on the pedal. Sorry if the name if confusing but it was the only field from 860C that I could reuse for this. This parameter does not change the maximum assistance provided for any selected level when pressure on pedal is maximum but it allows to increase (or decrease) the assistance when pressure on pedal is quite low.
This parameter can vary between -20 and +20. When this parameter is set on 0, the assistance is calculated based on the value of the torque sensor.
The more the parameter is higher than 0 (up to 20) the less assistance you will get for small pressure on the pedal (but so the more you get for highier pressure - still never exceeding the max value defined for the selected level). In other words, the ratio assistance per kg pressure is lower for lowest pressure and higher for highest pressure compared to parameter set on 0.
Reversely, the more the parameter is lower than 0 (up to -20), the more assistance you will get for small pressure on the pedal. In other words, the ratio assistance per kg pressure is higher for lowest pressure and lower for highest pressure compare to parameter set on 0.
* Pedal torque ADC angle adj is not used
* Pedal torque ADC offset (no weight) is very important. In TSDZ2 or in previous versions, the firmware read the torque sensor during the first 3 seconds and considers this value as the reference when no load is applied. This was done in order to get an automatic recalibration at each power on. This process is not good for TSDZ8 because, for some TSDZ8, the value varies significantly with the position of the pedal. So in this version of OSF, there is no autocalibration of the torque sensor with no load at power on. Instead, the user has to fill in "Pedal torque ADC offset" the value that will become the reference. To find the value to encode, you must let the display show the value of ADC torque sensor (see display manual).  When no load is applied on the pedal, turn manually the pedal and look at the values of "ADC torque sensor". Note the MAXIMUM. I expect that value should be between 150 and 190 depending on your motor. Then add some margin (e.g 10) to avoid assistance with very low pressure and enter the value in "Pedal torque ADC offset". You can adapt the "margin" value to your preference (a higher value will require more pressure on the pedal before getting assistance but will reduce consumption).
* Pedal torque ADC max (max weight) has to be filled : to find the value, look at field ADC torque sensor when you apply the max pressure on the pedal (about 80 kg = full human weight) while holding the brakes. It seems that the value should be around 450 for TSDZ8 which is quite different from TSDZ2.


Values for the different assist levels/modes are different from the default values for TSDZ2: TSDZ8 can provide more power than TSDZ2. In order to get access to the full power even for lower weight on the pedal the ratio value/assitance has been changed for TSDZ8. You have to use a lower value (2 X lower) to get the same assistance for Power, Torque and Hybrid assist modes.

Note : this version of OSF does not use anymore a parameter named "FOC multiplier". OSF calculates now the lead angle using the battery current and a table based on the motor speed (RPM). Furthermore, it applies automatically a correction to improve efficiency. So following text should be discarded (kept for history onlly): "OSF has a parameter named "FOC multiplier" that has an impact on motor efficency (for high current). Default value is 26 but it is not sure it is the best value. You can experiment other values (lower or higher) but be careful that motor efficiency can decrease a lot and the motor can become unstable and overheat. So be carreful when testing other values and start first with small changes. To test another value, you have to edit the file named "other settings/TSDZ8_heading.ini" in the line "foc_angle_multiplier". Replace the value "26" by another one, let javaconfigurator generate a new TSDZ8_config.hex file and upload it in the controller. I do not expect that the best value should exceed 40."


Note : there is no need to install all additional softwares mentioned in the manual as you will use only JavaConfigurator.jar and there is no need for compiling/flashing a STM microprocessor (on the opposite to TSDZ2).
So having java on you PC is enough.




Doubble click (at least on Windows) on JavaConfigurator.jar
It opens a graphical user interface. Take care to select "TSDZ8" as "Motor type". After editing your preferences, click the button "Compile HEX file". The configurator will generate a new file named "files_to_flash/TSDZ8_config.hex".

# 5.Flash the file with your configuraton

Flash the HEX file your created at step4 in the same way, you flashed already the firmware.
As said before, use menu "Manual programming" to avoid that J-flash totally erase the chip before flashing the configuration file. 

Note:
There is normally no need to re-flash the firmware each time you change the configuration file with other preferences.

Still if there are deep changes (in the firmware and/or the sheet), a "Main Version nr" in the JavaConfigurator will be changed.
When the controller starts running, it checks that the firmware is compatible with this version nr.
If not, it provides an error code E09 on the display (at least on VLCD5) and blocks the motor. It could be that the code is different (e.g. E08 on other displays). 

# 6. Fine tune some parameters (optional) + monitor (optional).
Note :  this readme section is not valid anymore for 0.1.18 (and upper) version. It should be updated. Still it gives some principles that can be usefull for debugging.

The motor use hall sensors to know the position of the rotor and synchronize the magnetic flux.
There can be minor differences in the positions, the sensitivity and the hysteresis of those sensors.
Combined with J-link, there is a tool allowing, while the motor is running :

- to monitor many values from the motor (current, speed, torque, ...)
- to change some parameters to fine tune the "global_offset_angle" that is used to position one hall sensor.

If you want to use this tool, you have to:

* install on you PC (windows) a tool uc_probe provided by Infineon at this link https://infineoncommunity.com/uC-Probe-XMC-software-download_ID712 . It requires that you fill in the register form to download it.

* once unzip and installed, run this software
* in to upper left corner, select settings and then select the options Jlink, 4000 kHz , SWD and little endian.
* in the menu File, select Open and in the file explorer, go to the folder where you put this TSDZ8 firmware and select the file "uc_probmonitoring.wspx"
* in the lowest panel named "Symbol browser", there should be a file name displayed ("mtb-example-xmc-gpio-toggle.elf"). It could be that it do not work the first time you try because I expect the software uses an absolute path to the file (and you are using your own path while I create the file with my path). In this case, click on the ELF button and it will open a popup to select the HEX file. In this popup, go to folder build/last_config and select the file named "mtb-example-xmc-gpio-toggle.elf". This step should not be required if you reopen the software later on.

* the central panel should now displays different fields and boxes.

* power on the motor (with the battery and the display) and connect the Jlink device between the motor and the PC.
* in uc_probe, in the upper left corner, click on the Run button (green arrow).
* you should see some values that this program collects from the controller. 

* the first block gives some general motor values. Do not modify them.

* the second gives some fields checked by the firmware to see is the motor is allowed to run. They can in many cases explain why the motor should not start running. Do not modify those fields

* the third block allows to activate/deactivate a special test mode.
    * when the button is OFF, the motor works as usual.
    * when the button is ON, the motor starts IMMEDIATELY running and discard (nearly) all parameters from other assist modes (power, torque, cadence, walk, ...) and their level of assist. You get full control of the power with just 2 parameters that you can modify with the sliders.
    * IMPORTANT NOTE : when button is ON, the motor can start running immediately. Disconnect the chain or be sure that the wheel can turn!!!!! 
    * the first parameter is the maximum current that the motor can drive during the test. Start with a low value and increase only if every thing seems fine.
    * the second parameter is the max duty cycle applied to the motor. This can be compared to the maximum percentage of the battery voltage that can be applied to the motor. The max value is 254 and correspond to 100%.
    * note that those 2 parameters are in fact used as input for some internal regulation. The real values applied on the motor depends also of other parameters like acceleration/deceleration that you can't control here. This can explain why the real current and duty cycle could differs from the value defined in the slider.
    * there are also 2 gauges (one up to 300mA and one up to 25000mA) to display the average current over the last second. As explain below, it allows to find the parameters providing the lowest current.

* the block fine tuning reference angle allows to modify the reference position of one hall sensor in order to optimise the motor. The purpose is to seach the value providing the lowest current when testing mode is activated AND the motor has no load (so best when chain is disconnected). The default value is 66. If you get better result with another value, you can caculate an offset to be fill in the field "global_offset_angle" from the file "other sttings/TSDZ8_header.ini" that exists where you put the javaConfigurator.rar. Take care that the value to fill must be (100 - 66 + the value providing the lowest current). So e.g. if lowest current is with reference angle = 65, the the value to fill is 100-66+65 = 99. It seems strange but it is done this way  in order to handle only positive values in the configurator. Afterwards, you have to rerun the javaConfigurator and to reflash the generated Hex file.

* the block "Fine tuning foc multiplier" allows to change a parameter that has a quite big impact on the motor efficiency when the motor is running fast and under load. This parameter is used to advance more or less the magnetic flux. To fine tune, best would be to install the motor on a bike home trainer, to select first a quite light load on the home trainer and to activate the testing mode with a  quite high current and duty cycle. Then Increase the load on the home trainer and start adjusting the slider in order to get the lowest current. The value providing best result has to be filled in the field "foc_angle_multiplier" from the file "other sttings/TSDZ8_header.ini" that exists where you put the javaConfigurator.rar. Afterwards, you have to rerun the javaConfigurator and to reflash the generated Hex file. 

* the block "Check best position for hall sensors" is for information. The positions/characteristics of hall sensors can slightly vary from motor to motor. The firmware tries to detect and adjust automatically the position of the sensors when the motor is running fast and at constant rpm. In those conditions, it takes one sensor (giving pattern1) as reference and calculates the position of the other patterns based on enlapsed time between changes. For technical reason, the angles are not in degree but in step (256 steps = 360°). The field "number of updates" indicates when the firmware is updating the angles because motor is running fast enough and at constant speed. Note: when reaching a limit, the counter restarts automatically from 0. So the value is not meaningful but only the fact that the value is changing. It is expected that the values does not differ to much from the default values at power on. 

* the block "Discard wheel speed sensor and force a fixed speed (km/h)" allows to use the tools without having a wheel speed sensor connected. In normal operations, the firmware gives an error (E08 on VLCD5 display - but could be different on other displays) when the motor runs for more than 12 sec and there is no incomming signal from the speed sensor. When you use the uc_probe tool, you have to connect the J-link device and so, usually, to disconnect the speed sensor. Setting a value greater than zero with the slider simulates a fixed speed and so avoid those errors.



# Developper

If you want, you can look at the software and modify it.

This software has been developped with 
 - Modus toolbox (from infineon)
 - Visual Studio Code (and some extensions).
Note: Segger Jlink is also used to flash the controller if you want to do it inside VS Code.

To install those firmwares, you have to follow the instructions provided in this link in the steps 1.1 and 1.2 (and 1.3 if you plan to use Jlink inside VS Code)
https://www.infineon.com/dgdl/Infineon-Visual-Studio-Code-user-guide-UserManual-v04_00-EN.pdf?fileId=8ac78c8c92416ca50192787be52923b2&redirId=248223

This can be quite long but is not very difficult.
There is no need to follow the instructions in chapters 2 and after.

Note: on my side I still had an issue to use Jlink to directly flash/debug the firmware and I had to
- rename the Jlink folder to remove the version nr (so it becomes just "SEEGER")
- edit the .vscode/settings.json file in this project to adapt the paths to Segger tools

Still take care to put all the folders inside an empty folder in the case you would like to develop/compile your self. The m 

Then there are some more steps to perform:
- open "modus-shell"  (this tool is part of the programs that have been dowloaded with modus toolbox)
- in modus-shell, make the folder named "OSF" the current folder (use "cd" commands)
- then enter in modus-shell the command "make getlibs" ; this will copy several libs
- then enter in modus shell the command "make vscode" ; this will generate/update some files in .vscode folder

Normally you are now ready to use VS Code and to compile.

To compiling the firmware:
* Open VS Code.
* In menu "File", select the option "Open Workspace from File..."
* Select a file named "TSDZ8.code-workspace" in folder "OSF".This should open all files.
* Select the menu "Terminal" and the option "Run Build Task"

Check if errors are generated. It is not abnormal to get quite many "Warnings" (but not "errors").
This step created a HEX file that can be flashed in the controller.
 
Note : it is also possible to flash the code directly from VS Code at the same time as you compile (e.g. to debug it) but this requires probably to modify some set up in .vscode/settings.json file in order to let VS Code knows the location of some Segger files (depend on the folder where you installed Segger Jlink).

Note: if you compile the firmware yourself, the generated hex file to upload is named "mtb-example-xmc-gpio-toggle.hex" and is present in
folder "test_gpio_in/build/tsdz8_for_GPIO_TEST/Debug" or in "test_gpio_in/build/last_config" (depending if you use the menu Run/start debugging or the menu Terminal/Run build task)