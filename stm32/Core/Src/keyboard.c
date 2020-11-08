#include "pin.h"
#include "stm32_handle.h"

uint8_t KeyGetRow()
{
	if (HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_RESET) {
		return 1;
	} else if (HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_RESET) {
		return 2;
	} else if (HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_RESET) {
		return 3;
	} else if (HAL_GPIO_ReadPin(ROW4_GPIO_Port, ROW4_Pin) == GPIO_PIN_RESET) {
		return 4;
  } else if(HAL_GPIO_ReadPin(ROW5_GPIO_Port, ROW5_Pin) == GPIO_PIN_RESET) {
		return 5;
  } else if (HAL_GPIO_ReadPin(ROW6_GPIO_Port, ROW6_Pin) == GPIO_PIN_RESET) {
		return 6;
  } else if (HAL_GPIO_ReadPin(ROW7_GPIO_Port, ROW7_Pin) == GPIO_PIN_RESET) {
		return 7;
	}
	return 0;  
}
int KeyScan()
{
	uint8_t keyMap[7][8] = {
		{0,},
		{0, 1,  7, 13, 18, 23, 28, 33},
		{0, 2,  8, 13, 19, 24, 29, 34},
		{0, 3,  9, 14, 20, 25, 30, 35},
		{0, 4, 10, 15, 21, 26, 31, 36},
		{0, 5, 11, 16, 22, 27, 32, 37},
		{0, 6, 12, 17}
	};
	HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
	
	uint8_t rowKey = KeyGetRow();
	if (rowKey != 0) {
		return keyMap[1][rowKey];
	}

	HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
	
	rowKey = KeyGetRow();
	if (rowKey != 0) {
		return keyMap[2][rowKey];
	}

	HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
	
	rowKey = KeyGetRow();
	if (rowKey != 0) {
		return keyMap[3][rowKey];
	}
	
	HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
	
	rowKey = KeyGetRow();
	if (rowKey != 0) {
		return keyMap[4][rowKey];
	}

	HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
	
	rowKey = KeyGetRow();
	if (rowKey != 0) {
		return keyMap[5][rowKey];
	}

	HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_RESET);
	
	rowKey = KeyGetRow();
	if (rowKey != 0) {
		return keyMap[6][rowKey];
	}

	
	return 0;
}
