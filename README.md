# Timer using STM32 Blue Pill
Implementing three popular methods of using timers (namely, Interrupt Mode, Polling Mode and DMA Mode). 

## Results:

Onboard LED control by Interrupt Mode.
Yellow LED (left) control by Polling Mode.
White LED (right) control by DMA Mode.

## 1. Using Timers in Interrupt Mode:
- It sets up a free-running counter.
- Update Event (UEV) flag is set when the timer reaches the Period value.
- The *TIM6_IRQHandler()* ISR fires when the timer overflows, and the *HAL_TIM_IRQHandler()* is then called.
- Example [[Timers using STM32#^a8c9a6]]

## 2. Using Timers in Poll Mode:
The idea behind the polling mode is that the timer counter register (*TIMx->CNT*) is accessed continuously to check for a given value.
A two common ways are:
1. checking if the timer current counter value is  *greater or equal* than the given value.
``` C++
...
while (1) {
	if(__HAL_TIM_GET_COUNTER(&htim2) >= 500){
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}
...
```

2. checking against the *UIF flag status*.
``` C++
...
while (1) {
if(__HAL_TIM_GET_FLAG(&tim) == TIM_FLAG_UPDATE) {
//Clear the IRQ flag otherwise we lose other events
__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
...
```

*Note: The correct way is by using Interrupt Mode* unless the timers runs very fast and interrupt (e.g. after every few microseconds) hinders the code execution.

## 3. Using Timers in DMA Mode:
This mode allows timers to trigger and control [[DMA]] transfers automatically, without CPU intervention. 

1. First, the DMA Handle is initialization.
```C++
  hdma_tim2_up.Instance = DMA1_Channel2;
  hdma_tim2_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim2_up.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim2_up.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim2_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tim2_up.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_tim2_up.Init.Mode = DMA_CIRCULAR;
  hdma_tim2_up.Init.Priority = DMA_PRIORITY_LOW;
```

2. A trigger event is first set for the DMA.  For example, we can define Timer UP request (generated when timer is overflowed) as an trigger event. 
```C++
   HAL_DMA_Init(&hdma_timx_up);
```

3. Then, the data (that is copied to the buffer of DMA) and the register (where the data is to be copied) are defined.
```C++
HAL_DMA_Start(&hdma_tim6_up, (uint32_t)data, (uint32_t)&GPIOA->ODR, 2);
```
where, '*hdma_tim6_up*' is the trigger, '*data*' is the address to data, '*GPIOA->ODR*' is the register, and '*2*' is the length of the array. 

4. Then, *TIM Update DMA* request is enabled.
```C++
__HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);
