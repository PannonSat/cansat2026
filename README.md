
# CanSat 2026 - Team PannonSat

## 🚀 About us 
We are highschoolers from the Benedictine School of Pannonhalma, competing in the CanSat Hungary 2026 competition, on this page, you will find our most recent sourcecode and documentation!

If you want to read more about the project itself, make sure to check out pannonsat.hu.

## 🪛 Hardware

We use the Arduino platform to manage all of our tasks. Our board (the Nano RP2040 Connect), comes with plenty of already well known microchips, sensors, so it's perfect to use for CanSat.

## 👾 Code

The flight software is sectioned into modules, for enhanced structure.
Let's look at some key elements of it:

1, Main:
- Main control logic
- Calls the necessarry functions at given times
- Operating mode control

2, Sensors:
- Each divided onto: init(), run(), get() functions
- Performs the necessarry calculation

3, Software Timer:
- From the Qpac-CanSat team
- Manages, times our Code

4, Other
- We have other key elements of the code, that work on it's own way, their documentation is in the code


## Author: 
Sárossy Illés - the code wizzard of PannonSat
