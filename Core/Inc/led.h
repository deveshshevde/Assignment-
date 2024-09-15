/*
 * led.h
 *
 *  Created on: Sep 15, 2024
 *      Author: deveshshevde
 */


#pragma once

#include "main.h"

void lcd_init (void);

void lcd_send_cmd (char cmd);

void lcd_send_data (char *data);

void lcd_send_string (char *str);

void lcd_put_cur(int row, int col);

void lcd_clear (void);
