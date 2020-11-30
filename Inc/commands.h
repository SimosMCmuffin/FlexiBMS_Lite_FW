#ifndef COMMANDS_H_
#define COMMANDS_H_

void commands_process_packet(uint8_t from, uint8_t *data, unsigned int len,
		void(*reply_func)(uint8_t to, uint8_t *data, unsigned int len));

#endif /* COMMANDS_H_ */
