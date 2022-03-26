/*
    ...

*/
#ifndef __FB_DATA_ACQ_H__
#define __FB_DATA_ACQ_H__

void dataAcq_init_gpio();
static void IRAM_ATTR gpio_isr_handler_catchHallS(void* flag);
void dataAcq_task(void* arg);
float dataAcq_get_yarn_tension(int adc_channel);
void dataAcq_calc_mean(dataAcq_t *pDataAcq, float fromDeg, float toDeg, int amount_of_samples);
void dataAcq_filter_data(dataAcq_t *pDataAcq);



#endif /* __FB_DATA_ACQ_H__ */