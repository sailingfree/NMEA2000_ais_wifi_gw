// Idle loop

void IdleInit();
void idleLoop(void * parameter);
int getCpuAvg(int core);
void calibrateCpu();

extern Stream * Console;