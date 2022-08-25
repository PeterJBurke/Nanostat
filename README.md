## Instructions for a fully functional nanostat system:

## Hardware order:
You can order the board basically fully assembled from JLCPBC, or other vendors/manufacturers. 

### Description of gerber and cad files:

The .json files contain the schematics and layout.
These can be read into EasyEDA, an online CAD program (https://easyeda.com/editor).
However, you can use any other CAD program you want. (e.g. Altium, Autodesk Eagle, etc.)
From within the CAD program, you can generate the BOM, pick and place file, and Gerber files. I have done that and placed those files in this github repo. (https://github.com/PeterJBurke/Nanostat). 
You can use any other manufacturer you want (e.g. JLCPBC, PCBWAY, 4PCB, etc.)

In general what is needed is the gerber file to manufacture the board, and then the components to solder into the board (BOM). If the soldering is done by pick and place (highly recommended as they are tiny), then the pick and place file is needed. Some manufacturers require you to source the parts separately (from e.g. Digikey, Mouser, Newark, etc.)  and ship to the manufacturer. Some manufacturers will source them for you from a 3rd party (from e.g. Digikey, Mouser, Newark, etc.). Some manufacturers (such as JLCPCB) have their own in house source of parts (warehouse, if you will), so you don’t need a 3rd party. 

Within the EasyEDA online cad program, it is possible to only choose components that are available, and in stock directly from JLCPCB for pick and place assembly. (Other CAD programs also have a way to interface to in stock components, but I find JLCPCB and EasyEDA to be the simplest and most elegant solution).  To place a component within EasyEDA, go to the schematic editor, right click, then place component from the drop down menu. A pop-up menu will show up, allowing you to search within the online library. You want components that are in stock, and can be assembled via SMT. So the class should be “JLCPCB assembled” and the SMT icon (SMT with square) as well as a “cart” icon for in stock should appear next to the part you have selected. There is “extended” and “standard”. You should try to pick “standard” as for pick and place SMT, these are free compared to the “extended”, which have to be manually loaded by JLCPCB into the cassette and therefore costs more. A good tutorial in YT is here: https://www.youtube.com/watch?v=MICBFN2mD6Q&t=727s


For the published version (3.5), the files are included for historical purposes.
Due to the supply chain issues and stock fluctuations, the parts were not in stock, so I have made a new version (3.5.1) which has parts in stock as of 8/22/2024. (Three capacitors were changed.)

It seems the LMP91000, the analog front end and the heart of the system, is not in stock in the USA. Digikey has zero in stock. Mouser, Newark have it on order, expected availability in summer 2022. So the small numbers that JLCPBC in stock are $60 each, about ten times what they were in 2021 when I bought them. This brings the cost from $25 for the whole board, to about $80-$90/board, with a minimum order of 5.

### Resources:

jlcpcb.com for ordering boards, including pick and place assembly, sourcing parts

https://easyeda.com/editor for cad design, layout and order to jlcpcb
 
Sourcing global parts for JLCPCB can be done, see:
https://support.jlcpcb.com/article/192-how-to-use-jlcpcb-global-sourcing-parts-service

Some useful tutorials:

Tutorial EasyEDA has their own YT channel: 
https://www.youtube.com/watch?v=MsuR6W-jN5M&list=PLbKMtvtYbdPMZfzGuVTdc0MWKrFvU4nsu

Online gerber viewer:
https://www.pcbway.com/project/OnlineGerberViewer.html


## After receiving hardware:

You will have 3 for the control, reference, and working electrode. The electrochemical cell is up to you. I use alligator clips, as shown in the paper, search “0.1" female to alligator clip”.

### Battery/power:
Choose the correct battery size as desired. Without battery, the NanoStat can be powered over the USB interface.

### Case:
3d print as desired. STL are on the github repo. (https://github.com/PeterJBurke/Nanostat)

## Software/firmware installation:

The software is developed in Microsoft Visual Studio Code. It uses PlatformIO. There are plenty of tutorials on this online. You will have to install this development environment, configure it, and download the source code as well as package configurations from github (https://github.com/PeterJBurke/Nanostat). After successful compilation, then you need to transfer the firmware hex code to the board using USB.

## Use:
Instructions are on the website hosted on the nanostat. See the paper (reference below) for examples of use cases.

### Getting to the website interface:
On first use, a wifi hot spot will be created by the Nanostat. Log into it and enter the wifi credentials you want for your local wifi network, and reboot. Then you can log into the Nanostat from your local wifi network. Try nanostat.local as the address in any browser. If that does not work, find the local IP address from your router.

### Firmware updates:
Can be done over wifi.


## Reference:
Shawn Chia-Hung Lee, Peter J. Burke “NanoStat: An open source, fully wireless potentiostat” Electrochimica Acta, https://doi.org/10.1016/j.electacta.2022.140481 (2022)


## License

This project is licensed under the MIT License - see the [LICENSE.txt](LICENSE.txt) file for details

## Acknowledgments

 * Many others have contributed to the source code that is used in this work. See references in the code for the packages used, as well as references in the published paper for additional acknowledgements.
