/*
	Copyright 2019 - 2021 Simo Sihvonen	"Simos MCmuffin" - simo.sihvonen@gmail.com

	This file is part of the FlexiBMS Lite firmware.

	The FlexiBMS Lite firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The FlexiBMS Lite firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIG_H_
#define CONFIG_H_

#define HW_NAME "FlexiBMSlite"

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 18

#define STM32_UUID_8 ((uint8_t*)0x1FFF7590)

#define CAN_ID 10	//non-static ID used now

#define MAX_CELLS 12

#define MAX_CELLS_BALANCING 5


#endif /* CONFIG_H_ */
