#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

void joystick_update(uint8_t read, uint16_t address, uint8_t *byte);
void joystick_scanline_update(uint8_t cycles);
void joystick_state_set(uint8_t btn0, uint8_t btn1, uint8_t pdl0, uint8_t pdl1);

#endif /* __JOYSTICK_H__ */
