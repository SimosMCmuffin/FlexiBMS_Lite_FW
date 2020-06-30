#ifndef COMMANDS_H_
#define COMMANDS_H_

void commands_process_packet(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len));

#endif /* COMMANDS_H_ */
