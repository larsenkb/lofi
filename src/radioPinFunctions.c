/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* Please define your platform spesific functions in this file ...
* -----------------------------------------------------------------------------
*/

#include <avr/io.h>

#define set_bit(reg,bit) reg |= (1<<bit)
#define clr_bit(reg,bit) reg &= ~(1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))

#define CE    0
#define CSN   1
#define SCK   4
#define MOSI  6
#define MISO  5

/* ------------------------------------------------------------------------- */
void nrf24_setupPins()
{
    set_bit(DDRA,CE); // CE output
    set_bit(DDRA,CSN); // CSN output
    set_bit(DDRA,SCK); // SCK output
    set_bit(DDRA,MOSI); // MOSI output
    clr_bit(DDRA,MISO); // MISO input
}
/* ------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
    if (state) {
        set_bit(PORTA,CE);
    } else {
        clr_bit(PORTA,CE);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
    if (state) {
        set_bit(PORTA,CSN);
    } else {
        clr_bit(PORTA,CSN);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_sck_digitalWrite(uint8_t state)
{
    if (state) {
        set_bit(PORTA,SCK);
    } else {
        clr_bit(PORTA,SCK);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_mosi_digitalWrite(uint8_t state)
{
    if (state) {
        set_bit(PORTA,MOSI);
    } else {
        clr_bit(PORTA,MOSI);
    }
}
/* ------------------------------------------------------------------------- */
uint8_t nrf24_miso_digitalRead()
{
    return check_bit(PINA,MISO);
}
/* ------------------------------------------------------------------------- */
