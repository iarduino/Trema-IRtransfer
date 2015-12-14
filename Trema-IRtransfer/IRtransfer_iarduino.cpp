#include "IRtransfer_iarduino.h"
volatile	IRtransfer_volatile_variable_class IRVVC;

//			инициализация ИК датчика
void		IRtransfer::begin(uint8_t i, uint8_t j, bool k){											//	(№ вывода подключенного к ИК датчику, № вывода подключённого к светодиоду)
			IRVVC.IR_pins_NUM_INPUT=i;																	//	сохраняем номер вывода к которому подключён выход ИК датчика
			IRVVC.IR_pins_NUM_OUTPUT=j;																	//	сохраняем номер вывода к которому подключён вход светодиода
			IRVVC.IR_flag_SEND_INVERT=k;																//	сохраняем флаг инверсии сигнала для светодиода
			pinMode(IRVVC.IR_pins_NUM_INPUT, INPUT);													//	переводим вывод ИК приемника в режим приёма
			IRVVC.IR_flag_READ_INVERT=digitalRead(IRVVC.IR_pins_NUM_INPUT);								//	определяем режим работы ИК датчика (прямой/инверсный)
			if(j<255){pinMode(IRVVC.IR_pins_NUM_OUTPUT, OUTPUT);}										//	переводим вывод светодиода в режим передачи
			if(j<255){digitalWrite(IRVVC.IR_pins_NUM_OUTPUT, IRVVC.IR_flag_SEND_INVERT);}				//	выводим 0 на светодиод (или 1 если установлен флаг инверсии)
			IR_flag_TOGGLE_SEND=0;																		//	флаг последнего состояния бита toggle
			IR_func_TIMER2_SETREG(20);																	//	устанавливаем режим работы таймера 2 на частоту прерываний по совпадению счётного регистра TCNT2 и регистра сравнения OCR2A = 20кГц
			IRVVC.IR_flag_SEND=0;																		//	устанавливаем режим работы функции обработки прерываний в чтение
}
//	проверка наличия принятого пакета
bool		IRtransfer::check(bool i){
//			отвечаем отсутствием новых не прочитанных пакетов
			if(!IRVVC.IR_uint_READ_STATUS){return false;}												//	нет принятых пакетов
			if(IRVVC.IR_flag_CHECK){return false;}														//	все принятые пакеты прочтены
//			ждём окончания получения данных
			if(IRVVC.IR_uint_READ_STATUS<4){while(IRVVC.IR_uint_READ_STATUS<4){;}}						//	принимаются первые пакеты, ждём...
//			сохраняем полученные данные (буферизируем)
			for(IR_var_I=0; IR_var_I<2;  IR_var_I++){
			for(IR_var_J=0; IR_var_J<68; IR_var_J++){
				IR_mass_PACK[IR_var_I][IR_var_J]=IRVVC.IR_mass_PACK[IR_var_I][IR_var_J];				//	сохраняем данные пакетов
			}	IR_uint_PACK_LEN[IR_var_I]=IRVVC.IR_uint_PACK_LEN[IR_var_I];							//	сохраняем длину  пакетов
			}
//			запрещаем повторное чтение уже полученного пакета
			if(i){if(IRVVC.IR_uint_READ_STATUS==6){IRVVC.IR_flag_CHECK=1;} if(IRVVC.IR_flag_READ_REPEAT){IRVVC.IR_flag_READ_REPEAT=0;}else{return false;}}else{IRVVC.IR_flag_CHECK=1;}	//	устанавливаем однократный вывод результата о пакетах
//			сброс установленного протокола
			if(!coding){IR_flag_SET_PROTOCOL=0;}
//			определяем протокол передачи данных, если он не указан пользователем
			if(!IR_flag_SET_PROTOCOL){coding=IR_func_PROTOCOL();}
//			раскодируем принятые данные в соответствии с протоколом
			do{	if(IR_func_DECODE(0)){data=IR_uint_DATA; length=IR_uint_LENGTH;}else{return false;}		//	информационный пакет
				if(IR_func_DECODE(1)){data_repeat=IR_uint_DATA; length_repeat=IR_uint_LENGTH;}			//	повторный пакет
//				если протокол не указан пользователем и был определён как NRC, но не прошел проверку, то меняем протокол на IR_BIPHASIC и повторяем раскодирование
				IR_var_J=0; if(coding==IR_NRC){if(!IR_flag_SET_PROTOCOL){if(!IR_func_CHECK_NRC()){IR_var_J=1; coding==IR_BIPHASIC;}}}
			}	while(IR_var_J);
//			определяем тип повторного пакета, если протокол передачи данных не указан пользователем
			if(!IR_flag_SET_PROTOCOL){
				if(IR_uint_PACK_LEN[1]==0)						{IR_uint_PACK_REPEAT_TYPE=0;}else		//	повторного пакета нет
				if(coding==IR_NRC)								{IR_uint_PACK_REPEAT_TYPE=3;}else		//	повторный пакет уникален
				if(length==length_repeat && data!=data_repeat)	{IR_uint_PACK_REPEAT_TYPE=1;}else		//	повторный пакет имеет инверсные биты
				if(length==length_repeat && data==data_repeat)	{IR_uint_PACK_REPEAT_TYPE=2;}else		//	повторный пакет идентичен информационному
																{IR_uint_PACK_REPEAT_TYPE=3;}			//	повторный пакет уникален
				IR_uint_PACK_REPEAT=IR_func_CREATE_PACK_REPEAT(false);									//	значение или биты инверсии для отправки пакета повтора
			}
			return true;
}
//			отправка пакета данных на светодиод
void		IRtransfer::send(uint32_t i, bool j){
			if(IRVVC.IR_pins_NUM_OUTPUT<255){
				uint32_t k=(IR_uint_PACK_PAUSE-2-(128000000/F_CPU))*20;										//	-2мс на задержки в функции IR_func_TIMER2_SETREG, -8мс для F_CPU=16МГц на установку регистров таймера в функции IR_func_TIMER2_SETREG, *20 преобразуем мс в мкс/50
				if(IRVVC.IR_uint_CALL_PAUSE>10){j=false;}													//	если пауза между вызовами данной функции больше 500мкс, то считаем что удержания не было
				if(IR_uint_DATA_PREVIOUS!=i){j=false;}														//	если команда для передачи отличается от предыдущей команды, то считаем что удержания не было
				if(j){	IR_func_DELAY(0,(coding==IR_NRC)?90:k);												//	задержка перед повторным пакетом
						IR_flag_EVEN_SEND=!IR_flag_EVEN_SEND;												//	определяем чётность пакета (1-чётный/0-нечётный)
						if(coding==IR_NRC || (IR_uint_PACK_REPEAT_TYPE==1 && !IR_flag_EVEN_SEND)){			//	для NRC повторным является информационный пакет, а для пакетов с инверсными битами - каждый нечётный пакет отправляется без инверсии
											IR_func_SEND(i,length);											//	отправка повторного пакета 
						}else{				IR_func_SEND(IR_func_CREATE_PACK_REPEAT(true,i),length_repeat);}//	отправка повторного пакета
				}else{	IR_flag_TOGGLE_SEND=!IR_flag_TOGGLE_SEND;											//	инвертируем состояние бита toggle
						IR_flag_EVEN_SEND=0;																//	указываем, что отправлен нечётный пакет
						if(coding==IR_NRC){	IR_func_SEND(IR_func_CREATE_PACK_REPEAT(true,i),length_repeat); IR_func_DELAY(0,k);}//	для NRC перед информационным отправляется стартовый пакет
											IR_func_SEND(i,length);											//	отправка информационного пакета
				}		IR_uint_DATA_PREVIOUS=i;															//	сохраняем значение отправленной команды
			}
}
//			загрузка протокола передачи данных
void		IRtransfer::protocol(char* i){
			IR_flag_SET_PROTOCOL		=	1;
			coding						=	i[ 0] & 0x3F;
			frequency					=	i[ 1] & 0x3F;
			length						=	i[ 2] & 0x3F;
			length_repeat				=	i[ 3] & 0x3F;
			IR_uint_PACK_PAUSE			=	i[ 4] & 0x3F | (i[21]&0x0C)<<4;
			IR_uint_START_PULSE			=	i[ 5] & 0x3F | (i[21]&0x03)<<6;
			IR_uint_START_PAUSE			=	i[ 6] & 0x3F | (i[22]&0x30)<<2;
			IR_uint_STOP_PULSE			=	i[ 7] & 0x3F | (i[22]&0x0C)<<4;
			IR_uint_STOP_PAUSE			=	i[ 8] & 0x3F | (i[22]&0x03)<<6;
			IR_uint_TOGGLE_PULSE		=	i[ 9] & 0x3F | (i[23]&0x30)<<2;
			IR_uint_TOGGLE_PAUSE		=	i[10] & 0x3F | (i[23]&0x0C)<<4;
			IR_uint_TOGGLE_POSITION		=	i[11] & 0x3F;
			IR_uint_BIT_PULSE_MAX		=	i[12] & 0x3F;
			IR_uint_BIT_PULSE_MIN		=	i[13] & 0x3F;
			IR_uint_BIT_PAUSE_MAX		=	i[14] & 0x3F;
			IR_uint_BIT_PAUSE_MIN		=	i[15] & 0x3F;
			IR_uint_PACK_REPEAT			=	i[16] & 0x3F; IR_uint_PACK_REPEAT<<=6;
			IR_uint_PACK_REPEAT			|=	i[17] & 0x3F; IR_uint_PACK_REPEAT<<=6;
			IR_uint_PACK_REPEAT			|=	i[18] & 0x3F; IR_uint_PACK_REPEAT<<=6;
			IR_uint_PACK_REPEAT			|=	i[19] & 0x3F; IR_uint_PACK_REPEAT<<=6;
			IR_uint_PACK_REPEAT			|=	i[20] & 0x3F; IR_uint_PACK_REPEAT<<=2;
			IR_uint_PACK_REPEAT			|= (i[21] & 0x30)>>4;
			IR_uint_PACK_REPEAT_TYPE	=	i[23] & 0x03;
			IR_flag_START				=	i[24] & 0x20 ? 1:0;
			IR_flag_STOP				=	i[24] & 0x10 ? 1:0;
			IR_flag_TOGGLE				=	i[24] & 0x08 ? 1:0;
}
//			выгрузка протокола передачи данных
char*		IRtransfer::protocol(){
			IR_char_PROTOCOL[ 0]		=	0x40 | (0x3F & coding					);					//	всего		6 бит номера кодировки
			IR_char_PROTOCOL[ 1]		=	0x40 | (0x3F & frequency				);					//	всего		6 бит частоты в кГц
			IR_char_PROTOCOL[ 2]		=	0x40 | (0x3F & length					);					//	всего		6 бит длинны информационного пакета в битах
			IR_char_PROTOCOL[ 3]		=	0x40 | (0x3F & length_repeat			);					//	всего		6 бит длинны пакета повтора в битах
			IR_char_PROTOCOL[ 4]		=	0x40 | (0x3F & IR_uint_PACK_PAUSE		);					//	последние	6 бит длительности паузы между пакетами в мс
			IR_char_PROTOCOL[ 5]		=	0x40 | (0x3F & IR_uint_START_PULSE		);					//	последние	6 бит длительности импульса бита старт в мкс/50
			IR_char_PROTOCOL[ 6]		=	0x40 | (0x3F & IR_uint_START_PAUSE		);					//	последние	6 бит длительности паузы бита страт в мкс/50
			IR_char_PROTOCOL[ 7]		=	0x40 | (0x3F & IR_uint_STOP_PULSE		);					//	последние	6 бит длительности импульса бита стоп в мкс/50
			IR_char_PROTOCOL[ 8]		=	0x40 | (0x3F & IR_uint_STOP_PAUSE		);					//	последние	6 бит длительности паузы бита стоп в мкс/50
			IR_char_PROTOCOL[ 9]		=	0x40 | (0x3F & IR_uint_TOGGLE_PULSE		);					//	последние	6 бит длительности импульса бита toggle/рестарт в мкс/50
			IR_char_PROTOCOL[10]		=	0x40 | (0x3F & IR_uint_TOGGLE_PAUSE		);					//	последние	6 бит длительности паузы бита toggle/рестарт в мкс/50
			IR_char_PROTOCOL[11]		=	0x40 | (0x3F & IR_uint_TOGGLE_POSITION	);					//	всего		6 бит позиции бита toggle/рестарт в пакете
			IR_char_PROTOCOL[12]		=	0x40 | (0x3F & IR_uint_BIT_PULSE_MAX	);					//	всего		6 бит максимальной длительности импульса бита в мкс/50
			IR_char_PROTOCOL[13]		=	0x40 | (0x3F & IR_uint_BIT_PULSE_MIN	);					//	всего		6 бит минимальной длительности импульса бита в мкс/50
			IR_char_PROTOCOL[14]		=	0x40 | (0x3F & IR_uint_BIT_PAUSE_MAX	);					//	всего		6 бит максимальной длительности паузы бита в мкс/50
			IR_char_PROTOCOL[15]		=	0x40 | (0x3F & IR_uint_BIT_PAUSE_MIN	);					//	всего		6 бит минимальной длительности паузы бита в мкс/50
			IR_char_PROTOCOL[16]		=	0x40 | (0x3F & IR_uint_PACK_REPEAT >> 26);					//	первые		6 бит данных для отправки пакета повтора
			IR_char_PROTOCOL[17]		=	0x40 | (0x3F & IR_uint_PACK_REPEAT >> 20);					//	вторые		6 бит данных для отправки пакета повтора
			IR_char_PROTOCOL[18]		=	0x40 | (0x3F & IR_uint_PACK_REPEAT >> 14);					//	третие		6 бит данных для отправки пакета повтора
			IR_char_PROTOCOL[19]		=	0x40 | (0x3F & IR_uint_PACK_REPEAT >>  8);					//	четвёртые	6 бит данных для отправки пакета повтора
			IR_char_PROTOCOL[20]		=	0x40 | (0x3F & IR_uint_PACK_REPEAT >>  2);					//	пятые		6 бит данных для отправки пакета повтора
			IR_char_PROTOCOL[21]		=	0x40 | (0x03 & IR_uint_PACK_REPEAT)<<4 | (IR_uint_PACK_PAUSE   >>6)<<2 | (IR_uint_START_PULSE>>6);	//	последние 2 бита IR_uint_PACK_REPEAT, первые 2 бита IR_uint_PACK_PAUSE, первые 2 бита IR_uint_START_PULSE
			IR_char_PROTOCOL[22]		=	0x40 | (IR_uint_START_PAUSE    >>6)<<4 | (IR_uint_STOP_PULSE   >>6)<<2 | (IR_uint_STOP_PAUSE >>6);	//	первые 2 бита IR_uint_START_PAUSE, первые 2 бита IR_uint_STOP_PULSE, первые 2 бита IR_uint_STOP_PAUSE
			IR_char_PROTOCOL[23]		=	0x40 | (IR_uint_TOGGLE_PULSE   >>6)<<4 | (IR_uint_TOGGLE_PAUSE >>6)<<2 | IR_uint_PACK_REPEAT_TYPE;	//	первые 2 бита IR_uint_TOGGLE_PULSE, первые 2 бита IR_uint_TOGGLE_PAUSE, 2 бита IR_uint_PACK_REPEAT_TYPE
			IR_char_PROTOCOL[24]		=	0x40 | IR_flag_START<<5 | IR_flag_STOP<<4 | IR_flag_TOGGLE<<3;	//	флаги
			IR_char_PROTOCOL[25]		=	0;
			return IR_char_PROTOCOL;
}
//			определение протокола передачи данных
uint8_t		IRtransfer::IR_func_PROTOCOL(){
			IR_var_J					=	0;															//	переменная для вывода номера протокола
			IR_uint_BIT_PULSE_MAX		=	0;															//	максимальная длинна импульсов
			IR_uint_BIT_PULSE_MIN		=	255;														//	минимальная  длинна импульсов
			IR_uint_BIT_PAUSE_MAX		=	0;															//	максимальная длинна пауз
			IR_uint_BIT_PAUSE_MIN		=	255;														//	минимальная  длинна пауз
			IR_flag_START				=	0;															//	флаг наличия сигнала старт
			IR_uint_START_PULSE			=	IR_mass_PACK[0][0];											//	предположим, что первый бит это сигнал старт
			IR_uint_START_PAUSE			=	IR_mass_PACK[0][1];											//	предположим, что первый бит это сигнал старт
			IR_flag_STOP				=	0;															//	флаг наличия сигнала стоп
			IR_uint_STOP_PULSE			=	IR_mass_PACK[0][IR_uint_PACK_LEN[0]-1];						//	предположим, что последний бит это сигнал стоп
			IR_uint_STOP_PAUSE			=	IR_mass_PACK[0][IR_uint_PACK_LEN[0]-2];						//	предположим, что последний бит это сигнал стоп
			IR_flag_TOGGLE				=	0;															//	флаг наличия импульса toggle/рестарт
			IR_uint_TOGGLE_PULSE_POS	=	0;															//	позиция бита toggle/рестарт в пакете
			IR_uint_TOGGLE_PAUSE_POS	=	0;															//	позиция бита toggle/рестарт в пакете
			IR_uint_TOGGLE_PULSE_SUM	=	0;															//	кол-во битов toggle/рестарт в пакете
			IR_uint_TOGGLE_PAUSE_SUM	=	0;															//	кол-во битов toggle/рестарт в пакете
			IR_uint_TOGGLE_PULSE		=	0;															//	длинна импульса toggle/рестарт
			IR_uint_TOGGLE_PAUSE		=	0;															//	длинна паузы toggle/рестарт
			IR_uint_PACK_REPEAT			=	0;															//	значение или биты инверсии для отправки пакета повтора
			IR_uint_PACK_REPEAT_TYPE	=	0;															//	тип повторного пакета
			IR_uint_PACK_PAUSE			=	IRVVC.IR_uint_PACK_PAUSE/20;								//	пауза между 1 и 2 пакетами в мс
			frequency					=	37;															//	несущая частота сигнала
			if(IR_uint_PACK_LEN[0]==1){					return IR_ONE_PULSE_LENGTH;}					//	если принят только один импульс в пакете, то возвращаем протокол: кодирование шириной одного импульса
			if(IR_uint_PACK_LEN[0]==3){IR_flag_START=1; return IR_ONE_PULSE_LENGTH;}					//	если принято только два импульса в пакете, то возвращаем протокол: кодирование шириной одного импульса
//			определяем максимальные и минимальные значения импульсов и пауз (без первого и последнего)
			for(IR_var_I=2; IR_var_I<IR_uint_PACK_LEN[0]-2; IR_var_I++){
				if(IR_var_I%2){	IR_uint_BIT_PAUSE_MAX=max(IR_uint_BIT_PAUSE_MAX, IR_mass_PACK[0][IR_var_I]);
								IR_uint_BIT_PAUSE_MIN=min(IR_uint_BIT_PAUSE_MIN, IR_mass_PACK[0][IR_var_I]);}
				else{			IR_uint_BIT_PULSE_MAX=max(IR_uint_BIT_PULSE_MAX, IR_mass_PACK[0][IR_var_I]);
								IR_uint_BIT_PULSE_MIN=min(IR_uint_BIT_PULSE_MIN, IR_mass_PACK[0][IR_var_I]);}
			}
//			проверяем наличие импульса toggle/рестарт
			IR_uint_TOGGLE_PULSE=IR_uint_BIT_PULSE_MAX; IR_uint_TOGGLE_PAUSE=IR_uint_BIT_PAUSE_MAX; IR_uint_BIT_PULSE_MAX=0; IR_uint_BIT_PAUSE_MAX=0;
			for(IR_var_I=2; IR_var_I<IR_uint_PACK_LEN[0]-2; IR_var_I++){
				if(IR_var_I%2){	if(IR_func_COMPARE(IR_mass_PACK[0][IR_var_I], IR_uint_TOGGLE_PAUSE, 6)){IR_uint_TOGGLE_PAUSE_POS=IR_var_I; IR_uint_TOGGLE_PAUSE_SUM++;}else{IR_uint_BIT_PAUSE_MAX=max(IR_uint_BIT_PAUSE_MAX, IR_mass_PACK[0][IR_var_I]);}}
				else{			if(IR_func_COMPARE(IR_mass_PACK[0][IR_var_I], IR_uint_TOGGLE_PULSE, 6)){IR_uint_TOGGLE_PULSE_POS=IR_var_I; IR_uint_TOGGLE_PULSE_SUM++;}else{IR_uint_BIT_PULSE_MAX=max(IR_uint_BIT_PULSE_MAX, IR_mass_PACK[0][IR_var_I]);}}
			}					if(IR_func_COMPARE(IR_uint_BIT_PULSE_MAX, IR_uint_BIT_PULSE_MIN, 6)||!IR_uint_BIT_PULSE_MAX||IR_uint_TOGGLE_PULSE_SUM!=1){IR_uint_BIT_PULSE_MAX=IR_uint_TOGGLE_PULSE;}else{IR_flag_TOGGLE=1;}
								if(IR_func_COMPARE(IR_uint_BIT_PAUSE_MAX, IR_uint_BIT_PAUSE_MIN, 6)||!IR_uint_BIT_PAUSE_MAX||IR_uint_TOGGLE_PAUSE_SUM!=1){IR_uint_BIT_PAUSE_MAX=IR_uint_TOGGLE_PAUSE;}else{IR_flag_TOGGLE=1;}
//			проверяем наличие сигналов старт и стоп
			if(IR_uint_START_PULSE>(IR_uint_BIT_PULSE_MAX+9) || (IR_uint_START_PULSE+9)<IR_uint_BIT_PULSE_MIN || IR_uint_START_PAUSE>(IR_uint_BIT_PAUSE_MAX+9) || (IR_uint_START_PAUSE+9)<IR_uint_BIT_PAUSE_MIN){IR_flag_START=1;}
			if(IR_func_COMPARE(IR_uint_BIT_PULSE_MAX, IR_uint_BIT_PULSE_MIN, 6) && !IR_func_COMPARE(IR_uint_BIT_PAUSE_MAX, IR_uint_BIT_PAUSE_MIN, 6)){IR_flag_STOP=1; if(IR_uint_STOP_PAUSE>IR_uint_BIT_PAUSE_MAX){IR_uint_STOP_PAUSE-=IR_uint_BIT_PAUSE_MAX;}else{IR_uint_STOP_PAUSE=0;}}else{IR_uint_BIT_PAUSE_MAX=max(IR_uint_BIT_PAUSE_MAX, IR_uint_STOP_PAUSE); IR_uint_BIT_PAUSE_MIN=min(IR_uint_BIT_PAUSE_MIN, IR_uint_STOP_PAUSE); IR_uint_BIT_PULSE_MAX=max(IR_uint_BIT_PULSE_MAX, IR_uint_STOP_PULSE); IR_uint_BIT_PULSE_MIN=min(IR_uint_BIT_PULSE_MIN, IR_uint_STOP_PULSE);}
//			проверяем наличие протоколов кодирования длинной импульса/паузы или бифазного кодирования
			if( IR_func_COMPARE(IR_uint_BIT_PULSE_MAX, IR_uint_BIT_PULSE_MIN, 6) && !IR_func_COMPARE(IR_uint_BIT_PAUSE_MAX, IR_uint_BIT_PAUSE_MIN, 6))																			{IR_var_J=IR_PAUSE_LENGTH;}	//	если длительность всех импульсов приблизительно равна, а длительность пауз      отличается,        то устанавливаем протокол: кодирование длинной паузы
			if(!IR_func_COMPARE(IR_uint_BIT_PULSE_MAX, IR_uint_BIT_PULSE_MIN, 6) &&  IR_func_COMPARE(IR_uint_BIT_PAUSE_MAX, IR_uint_BIT_PAUSE_MIN, 6))																			{IR_var_J=IR_PULSE_LENGTH;}	//	если длительность всех пауз      приблизительно равна, а длительность импульсов отличается,        то устанавливаем протокол: кодирование шириной импульса (ШИМ)
			if( IR_func_COMPARE(IR_uint_BIT_PULSE_MIN, IR_uint_BIT_PAUSE_MIN, 3) &&  IR_func_COMPARE(IR_uint_BIT_PULSE_MAX, IR_uint_BIT_PAUSE_MAX, 3))																			{IR_var_J=IR_BIPHASIC;}		//	если максимальные или минимальные значения длительностей и пауз приблизительно равны,              то устанавливаем протокол: кодирование длинной паузы
			if( IR_func_COMPARE(IR_uint_BIT_PULSE_MIN, IR_uint_BIT_PAUSE_MIN, 3) && (IR_func_COMPARE(IR_uint_BIT_PAUSE_MAX, IR_uint_BIT_PAUSE_MIN*2, 3) || IR_func_COMPARE(IR_uint_BIT_PULSE_MAX, IR_uint_BIT_PULSE_MIN*2, 3)))	{IR_var_J=IR_BIPHASIC;}		//	если минимальные значения импульсов и пауз приблизительно равны, а максимальные в два раза больше, то устанавливаем протокол: кодирование длинной паузы
//			проверяем наличие протоколов бифазного кодирования со специальными битами (toggle) или битами (старт/стоп)
			if(IR_var_J==IR_BIPHASIC){ IR_flag_STOP=0; IR_flag_TOGGLE=0;
				if(IR_func_COMPARE(IR_uint_BIT_PULSE_MIN, 17, 4) && !IR_flag_START){IR_var_J=IR_RS5; frequency=36;}
				if(IR_func_COMPARE(IR_uint_BIT_PULSE_MIN,  9, 4) &&  IR_flag_START && IR_func_COMPARE(IR_uint_START_PULSE, 54, 4) && IR_func_COMPARE(IR_uint_START_PAUSE, 17, 4)){IR_var_J=IR_RS6; frequency=36;}
				if(IR_func_COMPARE(IR_uint_BIT_PULSE_MIN, 10, 4) &&  IR_flag_START && IR_func_COMPARE(IR_uint_START_PULSE, 10, 4) && IR_func_COMPARE(IR_uint_START_PAUSE, 50, 4)){IR_var_J=IR_NRC; frequency=38;}
			}
			if(IR_var_J==IR_PAUSE_LENGTH){IR_uint_TOGGLE_POSITION=IR_uint_TOGGLE_PAUSE_POS-1;}
			if(IR_var_J==IR_PULSE_LENGTH){IR_uint_TOGGLE_POSITION=IR_uint_TOGGLE_PULSE_POS;}
			return IR_var_J;
}
//			раскодирование принятых данных в переменную IR_uint_DATA
bool		IRtransfer::IR_func_DECODE(uint8_t i){
			IR_uint_DATA=0; IR_uint_LENGTH=0;
			if(coding==IR_PAUSE_LENGTH		){for(IR_var_I=IR_flag_START?2:0; IR_var_I<IR_uint_PACK_LEN[i]-1; IR_var_I+=2){if(!(IR_flag_TOGGLE&&(IR_uint_TOGGLE_POSITION==IR_var_I))){IR_uint_DATA=(IR_uint_DATA<<1)+(IR_var_I<IR_uint_PACK_LEN[i]-3?(IR_mass_PACK[i][IR_var_I+1]>IR_uint_BIT_PAUSE_MIN+6):(IR_mass_PACK[i][IR_var_I+1]-IR_uint_STOP_PAUSE>IR_uint_BIT_PAUSE_MIN+6));	IR_uint_LENGTH++;}} return true;}
			if(coding==IR_PULSE_LENGTH		){for(IR_var_I=IR_flag_START?2:0; IR_var_I<IR_uint_PACK_LEN[i];   IR_var_I+=2){if(!(IR_flag_TOGGLE&&(IR_uint_TOGGLE_POSITION==IR_var_I))){IR_uint_DATA=(IR_uint_DATA<<1)+                                (IR_mass_PACK[i][IR_var_I  ]>IR_uint_BIT_PULSE_MIN+6);																				IR_uint_LENGTH++;}} return true;}
			if(coding==IR_ONE_PULSE_LENGTH	){IR_uint_DATA=IR_flag_START?IR_mass_PACK[i][2]:IR_mass_PACK[i][0]; IR_uint_LENGTH=1; return IR_uint_DATA>4?true:false;}
			if(coding==IR_NRC				){if(IR_func_DECODE_BIPHASIC(i?0:1,false)){return true;}else{if(IR_flag_SET_PROTOCOL || i){return false;}else{coding=IR_BIPHASIC;}}}
			if(coding==IR_RS5				){if(IR_func_DECODE_BIPHASIC(i    ,true )){return true;}else{if(IR_flag_SET_PROTOCOL || i){return false;}else{coding=IR_BIPHASIC;}}}
			if(coding==IR_RS5X				){if(IR_func_DECODE_BIPHASIC(i    ,true )){return true;}else{if(IR_flag_SET_PROTOCOL || i){return false;}else{coding=IR_BIPHASIC;}}}
			if(coding==IR_RS6				){if(IR_func_DECODE_BIPHASIC(i    ,false)){return true;}else{if(IR_flag_SET_PROTOCOL || i){return false;}else{coding=IR_BIPHASIC;}}}
			if(coding==IR_BIPHASIC			){if(IR_func_DECODE_BIPHASIC(i    ,true )){return true;}else{if(IR_flag_SET_PROTOCOL || i){return false;}else{coding=IR_BIPHASIC_INV;}}}
			if(coding==IR_BIPHASIC_INV		){if(IR_func_DECODE_BIPHASIC(i    ,false)){return true;}else{return false;}}
			return false;
}
//			декодирование бифазного кода
bool		IRtransfer::IR_func_DECODE_BIPHASIC(uint8_t i, bool j){
			IR_uint_DATA=0; IR_uint_LENGTH=0;
//			определяем наличие удвоенного бита toggle
			bool k=0; for(IR_var_I=IR_flag_START?2:0; IR_var_I<IR_uint_PACK_LEN[i]; IR_var_I+=2){if(IR_mass_PACK[i][IR_var_I]>IR_uint_BIT_PULSE_MAX+4 || IR_mass_PACK[i][IR_var_I+1]>IR_uint_BIT_PAUSE_MAX+4){k=1;}}
//			преобразуем данные в последовательность «1» и «0»
			IR_mass_BIPHASIC_BIN[0]=0; IR_mass_BIPHASIC_BIN[1]=0; IR_mass_BIPHASIC_BIN[2]=0; IR_mass_BIPHASIC_BIN[3]=0;
			for(IR_var_I=IR_flag_START?2:0, IR_var_J=0, IR_var_K=15; IR_var_I<IR_uint_PACK_LEN[i]; IR_var_I+=2){
				IR_mass_BIPHASIC_BIN[IR_var_J]|=1<<IR_var_K; if(IR_var_K){IR_var_K--;}else{IR_var_K=15; IR_var_J++;} IR_uint_LENGTH++; if(IR_mass_PACK[i][IR_var_I  ]>IR_uint_BIT_PULSE_MIN+5){if(coding!=IR_RS6 || !(IR_uint_LENGTH==9 || (IR_uint_LENGTH==10 && !k))){IR_mass_BIPHASIC_BIN[IR_var_J]|=1<<IR_var_K; if(IR_var_K){IR_var_K--;}else{IR_var_K=15; IR_var_J++;} IR_uint_LENGTH++;}}
				IR_mass_BIPHASIC_BIN[IR_var_J]|=0<<IR_var_K; if(IR_var_K){IR_var_K--;}else{IR_var_K=15; IR_var_J++;} IR_uint_LENGTH++; if(IR_mass_PACK[i][IR_var_I+1]>IR_uint_BIT_PAUSE_MIN+5){if(coding!=IR_RS6 || !(IR_uint_LENGTH==9 || (IR_uint_LENGTH==10 && !k))){IR_mass_BIPHASIC_BIN[IR_var_J]|=0<<IR_var_K; if(IR_var_K){IR_var_K--;}else{IR_var_K=15; IR_var_J++;} IR_uint_LENGTH++;}}
			}
//			сдвигаем данные на 1 бит вправо (если _/¯ = 1   ¯\_ = 0 )
			if(j){IR_func_DECODE_BIPHASIC_SHIFT(false); IR_uint_LENGTH++;}
//			корректируем количество битов до чётного числа
			if(IR_uint_LENGTH%2){IR_uint_LENGTH++;} if(!((IR_mass_BIPHASIC_BIN[(IR_uint_LENGTH-1)/16]&(1<<(16-(IR_uint_LENGTH-(((IR_uint_LENGTH-1)/16)*16)))))||(IR_mass_BIPHASIC_BIN[(IR_uint_LENGTH-1)/16]&(1<<(17-(IR_uint_LENGTH-(((IR_uint_LENGTH-1)/16)*16))))))){IR_uint_LENGTH-=2;}
//			убираем бит toggle
			IR_var_I=IR_mass_BIPHASIC_BIN[0]>>8; // сохраняем биты start и mode
			if(coding==IR_RS5	){IR_func_DECODE_BIPHASIC_SHIFT(true); IR_func_DECODE_BIPHASIC_SHIFT(true); IR_uint_LENGTH-=2; IR_var_I&=0b11110000; IR_mass_BIPHASIC_BIN[0]&=0b0000111111111111; IR_mass_BIPHASIC_BIN[0]|=IR_var_I<<8;}
			if(coding==IR_RS5X	){IR_func_DECODE_BIPHASIC_SHIFT(true); IR_func_DECODE_BIPHASIC_SHIFT(true); IR_uint_LENGTH-=2; IR_var_I&=0b11000000; IR_mass_BIPHASIC_BIN[0]&=0b0011111111111111; IR_mass_BIPHASIC_BIN[0]|=IR_var_I<<8;}
			if(coding==IR_RS6	){IR_func_DECODE_BIPHASIC_SHIFT(true); IR_func_DECODE_BIPHASIC_SHIFT(true); IR_uint_LENGTH-=2; IR_var_I&=0b11111111; IR_mass_BIPHASIC_BIN[0]&=0b0000000011111111; IR_mass_BIPHASIC_BIN[0]|=IR_var_I<<8;}
//			преобразуем последовательность «1» и «0» в число
			for(IR_var_I=0, IR_var_J=0, IR_var_K=15; IR_var_I<IR_uint_LENGTH; IR_var_I+=2){IR_uint_DATA<<=1;
				if(((IR_mass_BIPHASIC_BIN[IR_var_J]&(1<<IR_var_K))?1:0) && ((IR_mass_BIPHASIC_BIN[IR_var_J]&(1<<(IR_var_K-1)))?0:1)){IR_uint_DATA|=j?0:1;}else
				if(((IR_mass_BIPHASIC_BIN[IR_var_J]&(1<<IR_var_K))?0:1) && ((IR_mass_BIPHASIC_BIN[IR_var_J]&(1<<(IR_var_K-1)))?1:0)){IR_uint_DATA|=j?1:0;}else{return false;}
				if(IR_var_K>1){IR_var_K-=2;}else{IR_var_K=15; IR_var_J++;}
			}	IR_uint_LENGTH/=2;
			return true;
}
//			сдвиг данных на 1 бит (true - влево, false - вправо)
void		IRtransfer::IR_func_DECODE_BIPHASIC_SHIFT(bool i){
			if(i){	IR_mass_BIPHASIC_BIN[0]<<=1; IR_mass_BIPHASIC_BIN[0]|=IR_mass_BIPHASIC_BIN[1]>>15;
					IR_mass_BIPHASIC_BIN[1]<<=1; IR_mass_BIPHASIC_BIN[1]|=IR_mass_BIPHASIC_BIN[2]>>15;
					IR_mass_BIPHASIC_BIN[2]<<=1; IR_mass_BIPHASIC_BIN[2]|=IR_mass_BIPHASIC_BIN[3]>>15;
					IR_mass_BIPHASIC_BIN[3]<<=1; 
			}else{	IR_mass_BIPHASIC_BIN[3]>>=1; IR_mass_BIPHASIC_BIN[3]|=IR_mass_BIPHASIC_BIN[2]<<15;
					IR_mass_BIPHASIC_BIN[2]>>=1; IR_mass_BIPHASIC_BIN[2]|=IR_mass_BIPHASIC_BIN[1]<<15;
					IR_mass_BIPHASIC_BIN[1]>>=1; IR_mass_BIPHASIC_BIN[1]|=IR_mass_BIPHASIC_BIN[0]<<15;
					IR_mass_BIPHASIC_BIN[0]>>=1;
			}
}
//			проверка протокола передачи данных NRC по стартовому пакету
bool		IRtransfer::IR_func_CHECK_NRC(){				if(length_repeat<3){return false;}
			for(IR_var_I=1; IR_var_I<=length_repeat; IR_var_I++){
				if(IR_var_I==2 &&  (data_repeat & 1<<(length_repeat-IR_var_I))){return false;}
				if(IR_var_I!=2 && !(data_repeat & 1<<(length_repeat-IR_var_I))){return false;}
			}																	return true;
}
//			создание повторного пакета (0-для протокола/1-для отправки на светодиод, данные для светодиода)
uint32_t	IRtransfer::IR_func_CREATE_PACK_REPEAT(bool i, uint32_t j){ uint32_t k=0;
			switch(IR_uint_PACK_REPEAT_TYPE){
				case 1:	if(i)	{for(IR_var_I=1; IR_var_I<=length; IR_var_I++){k<<=1; k+=(j>>(length-IR_var_I)&1)?((IR_uint_PACK_REPEAT>>(length-IR_var_I)&1)?1:0):((IR_uint_PACK_REPEAT>>(length-IR_var_I)&1)?0:1);}}
						else	{for(IR_var_I=1; IR_var_I<=length; IR_var_I++){k<<=1; k+=((data>>(length-IR_var_I)&1)==(data_repeat>>(length-IR_var_I)&1));}} return k; break;
				case 2: return i?j:0; break;
				case 3: return i?IR_uint_PACK_REPEAT:data_repeat; break;
				default: return 0;
			}
}
//			сравнение чисел с указанием допуска	(число, число, допуск±)
bool		IRtransfer::IR_func_COMPARE(uint8_t i, uint8_t j, uint8_t k){
			int result = i - j; if(result<0){result*=-1;}
			return result>k? false:true;
}
//			передача пакета данных на светодиод
void		IRtransfer::IR_func_SEND(uint32_t i, uint8_t j){
//			определяем позицию бита toggle/рестарт
			IR_var_K=255; if(coding==IR_PAUSE_LENGTH || coding==IR_PULSE_LENGTH){if(IR_flag_TOGGLE){IR_var_K=IR_uint_TOGGLE_POSITION/2;} if(IR_flag_START){IR_var_K--;}}else if(coding==IR_RS5){IR_var_K=2;}else if(coding==IR_RS5X){IR_var_K=1;}else if(coding==IR_RS6){IR_var_K=4;}
//			устанавливаем таймер на частоту передачи
			if(frequency){IRVVC.IR_flag_SEND=1; IR_func_TIMER2_SETREG(frequency*2);}
//			отправляем сигнал старт
			if(IR_flag_START){IR_func_DELAY(1,IR_uint_START_PULSE); IR_func_DELAY(0,IR_uint_START_PAUSE);}
//			отправляем биты данных
			for(IR_var_I=0; IR_var_I<j; IR_var_I++){
//				определяем значение отправляемого бита
				IR_var_J = i>>(j-IR_var_I-1) & 1;
//				передаём бит toggle/рестарт
				if(IR_var_I==IR_var_K){
					switch(coding){
						case IR_PAUSE_LENGTH:							IR_func_DELAY(1						, IR_uint_TOGGLE_PULSE);
																		IR_func_DELAY(0						, IR_uint_TOGGLE_PAUSE);	break;
						case IR_PULSE_LENGTH:							IR_func_DELAY(0						, IR_uint_TOGGLE_PAUSE);
																		IR_func_DELAY(1						, IR_uint_TOGGLE_PULSE);	break;
						case IR_RS5: case IR_RS5X:						IR_func_DELAY(!IR_flag_TOGGLE_SEND	, IR_uint_BIT_PAUSE_MIN);
																		IR_func_DELAY( IR_flag_TOGGLE_SEND	, IR_uint_BIT_PAUSE_MIN);	break;
						case IR_RS6:									IR_func_DELAY( IR_flag_TOGGLE_SEND	, IR_uint_BIT_PAUSE_MIN*2);
																		IR_func_DELAY(!IR_flag_TOGGLE_SEND	, IR_uint_BIT_PAUSE_MIN*2);	break;
				}	}
//				передаем информационный бит
					switch(coding){
						case IR_PAUSE_LENGTH:							IR_func_DELAY(1			, IR_uint_BIT_PULSE_MIN);
																		IR_func_DELAY(0			, IR_var_J?IR_uint_BIT_PAUSE_MAX:IR_uint_BIT_PAUSE_MIN); break;
						case IR_PULSE_LENGTH:							IR_func_DELAY(0			, IR_var_J?IR_uint_BIT_PULSE_MAX:IR_uint_BIT_PULSE_MIN);
																		IR_func_DELAY(1			, IR_uint_BIT_PAUSE_MIN); break;
						case IR_ONE_PULSE_LENGTH:						IR_func_DELAY(1			, i); break;
						case IR_BIPHASIC: case IR_RS5: case IR_RS5X:	IR_func_DELAY(!IR_var_J	, IR_uint_BIT_PAUSE_MIN);
																		IR_func_DELAY( IR_var_J	, IR_uint_BIT_PAUSE_MIN); break;
						case IR_BIPHASIC_INV: case IR_NRC: case IR_RS6:	IR_func_DELAY( IR_var_J	, IR_uint_BIT_PAUSE_MIN);
																		IR_func_DELAY(!IR_var_J	, IR_uint_BIT_PAUSE_MIN); break;
					}
			}
//			отправляем сигнал стоп
			if(IR_flag_STOP){IR_func_DELAY(0,IR_uint_STOP_PAUSE); IR_func_DELAY(1,IR_uint_STOP_PULSE);}
//			устанавливаем на выходе уровень не активного состояния
			IRVVC.IR_pins_SEND_STATUS=0; digitalWrite(IRVVC.IR_pins_NUM_OUTPUT, IRVVC.IR_flag_SEND_INVERT?!i:i);
//			устанавливаем таймер на частоту приёма
			IR_func_TIMER2_SETREG(20); IRVVC.IR_flag_SEND=0;
}
//			передача импульса или паузы на светодиод
void		IRtransfer::IR_func_DELAY(bool i, uint32_t j){
			uint32_t k=micros(); if(frequency){IRVVC.IR_pins_SEND_STATUS=i; IRVVC.IR_pins_SEND=1;}else{digitalWrite(IRVVC.IR_pins_NUM_OUTPUT, IRVVC.IR_flag_SEND_INVERT?!i:i);}
			j*=50; j+=frequency?-30:50; k+=j; while(micros()<=k){}
}
//			установка значений регистров таймера под нужную частоту
void		IRtransfer::IR_func_TIMER2_SETREG(uint32_t i){i*=1000;
			if(F_CPU/255/  1<i){IR_var_I=   1; IR_var_J=1;}else											//	определяем значение предделителя IR_var_I
			if(F_CPU/255/  8<i){IR_var_I=   8; IR_var_J=2;}else											//	и значение битов регистра TCCR2B: CS22-CS20 = IR_var_J
			if(F_CPU/255/ 32<i){IR_var_I=  32; IR_var_J=3;}else
			if(F_CPU/255/ 64<i){IR_var_I=  64; IR_var_J=4;}else
			if(F_CPU/255/128<i){IR_var_I= 128; IR_var_J=5;}else
			if(F_CPU/255/256<i){IR_var_I= 256; IR_var_J=6;}else{IR_var_I=1024; IR_var_J=7;}
			TCCR2A	= 0<<COM2A1	| 0<<COM2A0	| 0<<COM2B1	| 0<<COM2B0	| 1<<WGM21	| 0<<WGM20;				//	биты COM2... = «0» (каналы A и B таймера отключены), биты WGM21 и WGM20 = «10» (таймер 2 в режиме CTC)
			TCCR2B	= 0<<FOC2A	| 0<<FOC2B	| 0<<WGM22	| IR_var_J;										//	биты FOC2... = «0» (без принудительной установки результата сравнения), бит WGM22 = «0» (таймер 2 в режиме CTC), биты CS22-CS20 = IR_var_J (значение предделителя)
			OCR2A	= (uint8_t)(F_CPU/(IR_var_I*i))-1;													//	значение регистра сравнения OCR2A настраивается под частоту переполнения счётного регистра TCNT2=i.  i=F_CPU/(предделитель*(OCR2A+1)) => OCR2A = (F_CPU/(предделитель*i))-1
			TIMSK2	= 0<<OCIE2B	| 1<<OCIE2A	| 0<<TOIE2;													//	разрешаем прерывание по совпадению счётного регистра TCNT2 и регистра сравнения OCR2A
			SREG	= 1<<7;																				//	устанавливаем флаг глобального разрешения прерываний 
			delay(1); IRVVC.IR_pins_READ=IRVVC.IR_pins_SEND=IRVVC.IR_uint_READ_STATUS=IRVVC.IR_pins_SEND_STATUS=IRVVC.IR_flag_CHECK=IRVVC.IR_flag_READ_REPEAT=IRVVC.IR_flag_PULSE=IRVVC.IR_uint_PACK_LENGTH=IRVVC.IR_uint_CALL_PAUSE=IRVVC.IR_uint_PACK_PAUSE=IRVVC.IR_uint_PACK_INDEX=IRVVC.IR_uint_PACK_NUM=IRVVC.IR_uint_PACK_LEN[0]=IRVVC.IR_uint_PACK_LEN[1]=0;
}

/* ISR */	ISR(TIMER2_COMPA_vect){
			if(IRVVC.IR_flag_SEND){
				if(IRVVC.IR_pins_SEND_STATUS){IRVVC.IR_pins_SEND=!IRVVC.IR_pins_SEND;}else{IRVVC.IR_pins_SEND=IRVVC.IR_flag_SEND_INVERT;}
				digitalWrite(IRVVC.IR_pins_NUM_OUTPUT, IRVVC.IR_pins_SEND);
			}else{
				IRVVC.IR_pins_READ=digitalRead(IRVVC.IR_pins_NUM_INPUT);								//	читаем состояние вывода в переменную IR_pins_READ
				if (IRVVC.IR_flag_READ_INVERT){IRVVC.IR_pins_READ=IRVVC.IR_pins_READ?0:1;}				//	инвертируем переменную IR_pins_READ, если установлен флаг инверсии сигналов
				if (IRVVC.IR_uint_READ_STATUS!=1 && IRVVC.IR_uint_READ_STATUS!=3){						//	если пакеты 1 или 2 не принимаются в данный момент
					if (IRVVC.IR_pins_READ){															//	и появился импульс, то он является первым в пакете
						if(IRVVC.IR_uint_READ_STATUS==2){IRVVC.IR_uint_PACK_PAUSE=IRVVC.IR_uint_PACK_LENGTH;}	//	сохраняем паузу между 1 и 2 пакетами
						IRVVC.IR_flag_PULSE			= 1;												//	устанавливаем флаг состояния сигнала в данный момент времени в 1 - PULSE; (0-PAUSE)
						IRVVC.IR_uint_PACK_INDEX	= 0;												//	устанавливаем индекс в массиве данных в 0 - первый бит пакета
						IRVVC.IR_uint_PACK_LENGTH	= 0;												//	устанавливаем длительность текущего импульса в 0
						if(IRVVC.IR_uint_READ_STATUS==6){IRVVC.IR_uint_READ_STATUS=0;}					//	устанавливаем статус 0, считаем что после паузы в 200мс нажата новая кнопка пульта
						if(IRVVC.IR_uint_READ_STATUS==0){IRVVC.IR_uint_PACK_NUM=0; IRVVC.IR_uint_PACK_LEN[0]=0; IRVVC.IR_uint_PACK_LEN[1]=0; IRVVC.IR_flag_CHECK=0; IRVVC.IR_flag_READ_REPEAT=1;}else	//	устанавливаем номер массива в 0 - первый пакет, обнуляем длину первого и второго пакета
						if(IRVVC.IR_uint_READ_STATUS==2){IRVVC.IR_uint_PACK_NUM=1;}						//	устанавливаем номер массива в 1 - второй пакет
						if(IRVVC.IR_uint_READ_STATUS<5){IRVVC.IR_uint_READ_STATUS++;}					//	устанавливаем очередной статус состояния приёма
					}else{																				//	и импульс не появляется
						if(IRVVC.IR_uint_READ_STATUS>0){IRVVC.IR_uint_PACK_LENGTH++;}					//	инкрементируем данные о длительности паузы
						else{if(IRVVC.IR_uint_CALL_PAUSE<255){IRVVC.IR_uint_CALL_PAUSE++;}}				//	инкрементируем паузу между вызовами функции send
						if(IRVVC.IR_uint_PACK_LENGTH>IR_INTERVAL_PRESS*20){IRVVC.IR_uint_READ_STATUS=6;}//	если длительность паузы превышает минимальный интервал между нажатиями клавиш, устанавливаем статус 6
						if(IRVVC.IR_uint_PACK_LENGTH>60000){IRVVC.IR_uint_READ_STATUS=0;}				//	если длительность паузы превышает 3сек устанавливаем статус 0
						if(IRVVC.IR_uint_READ_STATUS==5){												//	если уже принято 2 и более пакетов
							if(IRVVC.IR_uint_PACK_LENGTH==IR_INTERVAL_PACK*20){							//	если длительность паузы равна минимальному интервалу между пакетами
								IRVVC.IR_flag_READ_REPEAT=1;
							}
						}
					}
				}
				if (IRVVC.IR_uint_READ_STATUS==1 || IRVVC.IR_uint_READ_STATUS==3){						//	если принимается 1ый или 2ой пакет
					if (IRVVC.IR_pins_READ==IRVVC.IR_flag_PULSE){										//	если продолжается прием предыдущего импульса или паузы
						IRVVC.IR_uint_PACK_LENGTH++;													//	инкрементируем данные о длительности импульса или паузы
						if (!IRVVC.IR_flag_PULSE){														//	если принимается пауза
							if(IRVVC.IR_uint_PACK_LENGTH>IR_INTERVAL_PACK*20){							//	и её длительность превышает минимальный интервал между пакетами
								IRVVC.IR_uint_READ_STATUS++;											//	считаем что пакет принят
								IRVVC.IR_uint_PACK_LEN[IRVVC.IR_uint_PACK_NUM]=IRVVC.IR_uint_PACK_INDEX;//	сохраняем длину пакета
							}
						}
					}else{																				//	если импульс сменился на паузу, или наоборот
						IRVVC.IR_flag_PULSE=IRVVC.IR_pins_READ;											//	указываем что состояние сигнала в данный момент времени равно состоянию IR_pins_READ
						if (!IRVVC.IR_uint_PACK_INDEX<68){												//	если пакет не превышает длину массива
							IRVVC.IR_mass_PACK[IRVVC.IR_uint_PACK_NUM][IRVVC.IR_uint_PACK_INDEX]=IRVVC.IR_uint_PACK_LENGTH;	//	записываем длину импульса или паузы в массив
							IRVVC.IR_uint_PACK_LENGTH=0;												//	обнуляем длительность импульса или паузы
							IRVVC.IR_uint_PACK_INDEX++;													//	увеличиваем индекс очередного элемента в массиве
						}
					}
				}
			}
}
