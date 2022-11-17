![Overview](https://github.com/PeterJBurke/Nanostat/blob/master/GerberFiles/NanostatPictures.png)

## Instructions for a fully functional NanoStat system

## Hardware order
You can order the board basically fully assembled from JLCPBC, or other vendors/manufacturers. 
Here is a tutorial video showing how to order the hardware, from start to finish, using the github files:
https://www.youtube.com/watch?v=MpEoAoR6BOA

### Description of gerber and cad files

The .json files contain the schematics and layout.
These can be read into EasyEDA, an online CAD program (https://easyeda.com/editor).
However, you can use any other CAD program you want. (e.g. Altium, Autodesk Eagle, etc.)
From within the CAD program, you can generate the BOM, pick and place file, and Gerber files. I have done that and placed those files in this github repo. (https://github.com/PeterJBurke/Nanostat). 
You can use any other manufacturer you want (e.g. JLCPBC, PCBWAY, 4PCB, etc.)

In general what is needed is the gerber file to manufacture the board, and then the components to solder into the board (BOM). If the soldering is done by pick and place (highly recommended as they are tiny), then the pick and place file is needed. Some manufacturers require you to source the parts separately (from e.g. Digikey, Mouser, Newark, etc.)  and ship to the manufacturer. Some manufacturers will source them for you from a 3rd party (from e.g. Digikey, Mouser, Newark, etc.). Some manufacturers (such as JLCPCB) have their own in house source of parts (warehouse, if you will), so you don’t need a 3rd party. 

Within the EasyEDA online cad program, it is possible to only choose components that are available, and in stock directly from JLCPCB for pick and place assembly. (Other CAD programs also have a way to interface to in stock components, but I find JLCPCB and EasyEDA to be the simplest and most elegant solution).  To place a component within EasyEDA, go to the schematic editor, right click, then place component from the drop down menu. A pop-up menu will show up, allowing you to search within the online library. You want components that are in stock, and can be assembled via SMT. So the class should be “JLCPCB assembled” and the SMT icon (SMT with square) as well as a “cart” icon for in stock should appear next to the part you have selected. There is “extended” and “standard”. You should try to pick “standard” as for pick and place SMT, these are free compared to the “extended”, which have to be manually loaded by JLCPCB into the cassette and therefore costs more. A good tutorial in YT is here: https://www.youtube.com/watch?v=MICBFN2mD6Q&t=727s

### Supply chain issues

For the published version (3.5), the files are included for historical purposes.
Due to the supply chain issues and stock fluctuations, the parts were not in stock, so I have made a new version (3.5.1) which has parts in stock as of 8/22/2024. (The USB connector and LDO were changed.)

It seems the LMP91000, the analog front end and the heart of the system, is not in stock in the USA. Digikey has zero in stock. Mouser, Newark have it on order, expected availability in summer 2023. So the small numbers that JLCPBC in stock are $60 each, about ten times what they were in 2021 when I bought them. This brings the cost from $25 for the whole board, to about $80-$90/board, with a minimum order of 5. Hopefully when the chip shortage eases, the LMP91000 price will come back down to historical levels, and NanoStat boards will drop back down also.

### Resources

jlcpcb.com for ordering boards, including pick and place assembly, sourcing parts

https://easyeda.com/editor for cad design, layout and order to jlcpcb
 
Sourcing global parts for JLCPCB can be done, see:
https://support.jlcpcb.com/article/192-how-to-use-jlcpcb-global-sourcing-parts-service

Some useful tutorials:

EasyEDA online editor has their own YT channel: 
https://www.youtube.com/watch?v=MsuR6W-jN5M&list=PLbKMtvtYbdPMZfzGuVTdc0MWKrFvU4nsu

Online gerber viewer:
https://www.pcbway.com/project/OnlineGerberViewer.html

![3d model](https://github.com/PeterJBurke/Nanostat/blob/master/GerberFiles/3dmodel.png)

## After receiving hardware

You will have 3 pins for the control, reference, and working electrode. The electrochemical cell is up to you. I use alligator clips, as shown in the paper, search “0.1" female to alligator clip”.

There are also two pins for the battery, if you choose to use it. Make sure to get the correct polarity.

### Battery/power

Choose the correct battery size as desired. Without battery, the NanoStat can be powered over the USB interface.

### Case
3d print as desired. STL are on the github repo. (https://github.com/PeterJBurke/Nanostat)

![Lab hardware](https://github.com/PeterJBurke/Nanostat/blob/master/GerberFiles/NanostatCartoon.png)

## Software/firmware installation

For a tutorial on how to install the firmware, see:

https://youtu.be/zEnHAq8oCHY

The software is developed in Microsoft Visual Studio Code. It uses PlatformIO. There are plenty of tutorials on this online. You will have to install this development environment, configure it, and download the source code as well as package configurations from github (https://github.com/PeterJBurke/Nanostat). After successful compilation, then you need to transfer the firmware hex code to the board using USB.

You also need to transfer the website files in the data directory of this repo. This includes all the html files and js (javascript) files that serve up the website. ESP32 has a disk structure called "SPIFFS". For some tutorials on this see:
https://www.youtube.com/watch?v=Pxpg9eZLoBI
and
https://www.youtube.com/watch?v=NVD46mRbVXM

### Brief instructions starting from fresh linux OS
Update OS:

sudo apt-get update

sudo apt-get upgrade

Install git:

sudo apt-get install git

Install Visual studio code from website.

Within Visual studio code, install PlatformIO and suggested packages (C++ etc).

If it complains about a python program that is missing, run:

sudo apt-get install python3-venv

Now clone the git repo to your local machine within VS code.

https://github.com/PeterJBurke/Nanostat

Connect your Nanostat device to the USB port.

In Linux, it should auto-detect the device and you don't need to worry about drivers, etc.

However, you have to give permission to it.

Typically, it will called /dev/ttyUSB0 or something like that.

ls /dev/ttyUSB*

tells you which is listed.

Now give linux permission:

sudo chmod a+rw /dev/ttyUSB0

Now go back to VS Code.

I recommend to erase the flash memory on the nanostat, so we are starting with a clean system.

Compile and upload the firmware.

Upload the "filesystem". This uploads the website files.

Now power the nanostat off then on. You can power it from any USB power source (including the computer).
You don't need the computer connection anymore.

It will create a wifi hotspot called "nanostat".

Go to it, connect, and then point your browers to:

http://192.168.4.1

Enter your wifi credentials, then press reboot.

After reboot, rejoin your local wifi with your computer/smartphone/tablet, and browse to:

http://nanostat.local

If that fails, log into your router to find the locally assigned IP address.
Usually it is something like 192.168.1.xxx or 192.168.2.xxx etc, where xxx is between 1 and 256.

You should see the nanostat UI.


## Use
Instructions are on the website hosted on the nanostat. See the paper (reference below) for examples of use cases.

### Getting to the website interface
On first use, a wifi hot spot will be created by the Nanostat. It is call "NanostatAP". Connect to it with your pc or smartphone. The default IP address is 192.168.4.1. Go to that address, and it should open up a configuration page for wifi access.

Enter the wifi credentials you want for your local wifi network, and reboot. Then you can log into the Nanostat from your local wifi network. Try nanostat.local as the address in any browser. If that does not work, find the local IP address from your router.

## Troubleshooting access point

If you connect to the access point, make sure your computer has an ip address assigned:
MAC:
ifconfig
LINUX:
ifconfig
MSDOS:
ipconfig /all

These show assigned IP address of your local computer.
(Be sure you are reading it for the wifi adapter.)

It should be 192.168.4.xxx, where xxx is a number between 2 and 256.

### Firmware updates
Can be done over wifi. Instructions on website.


## Reference
Shawn Chia-Hung Lee, Peter J. Burke “NanoStat: An open source, fully wireless potentiostat” Electrochimica Acta, https://doi.org/10.1016/j.electacta.2022.140481 (2022)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Acknowledgments

 * Many others have contributed to the source code that is used in this work. See references in the code for the packages used, as well as references in the published paper for additional acknowledgements.
