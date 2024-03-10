# pico-iie

"Apple //e" emulator that runs on the Pi Pico.

Forked from https://github.com/pyrex8/pico-iie to adapt to ZX Murmulator dev.board.

# Hardware needed
To get it working you should have an Murmulator (development) board with VGA output. Schematics available here at https://github.com/AlexEkb4ever/MURMULATOR_classical_scheme
![Murmulator Schematics](https://github.com/javavi/pico-infonesPlus/blob/main/assets/Murmulator-1_BSchem.JPG)


# Периферия
<ul>
<li>VGA-copatible output with 222 color maping from the "Apple //e" 16-colors</li>
<li>Звук от пищалки выводится в виде 8-ми битного ШИМ.</li>
<li>PS/2 клавиатура</li>
<li>Dendy джойстики TBA</li>
<li>Wii джойстик TBA</li>
</ul>


### Compiling and Downloading to Pi Pico

There are good resources on the web for compiling and downloading C Pi Pico projects. The short version of this after git clone(ing) this project and setting up the tool chain navigate to the ```build``` folder. In build folder:

```
export PICO_SDK_PATH=../../pico-sdk
cmake ..
make
```


### Emulator Simplification

There are some simplifications/limitations to the emulator.

- Only emulates a 48K RAM.
- TEXT and HIRES modes only, no LORES mode.
- No blinking text, just NORMAL and INVERSE. FLASH displays as inverse and some odd characters.
- Most of the soft switch read "side affect" are not emulated.
- Vertical blanking register is not updated. This is due to it only being in the IIe so most games don't use it.

TBA