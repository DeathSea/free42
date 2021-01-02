#include "pin.h"
#include "stm32_handle.h"

uint32_t KeyGetRow()
{
    uint32_t row = 0;
    if (HAL_GPIO_ReadPin(ROW1_GPIO_Port, ROW1_Pin) == GPIO_PIN_RESET) {
        row |= 0x01; // 1
    } else if (HAL_GPIO_ReadPin(ROW2_GPIO_Port, ROW2_Pin) == GPIO_PIN_RESET) {
        row |= 0x02; // 2
    } else if (HAL_GPIO_ReadPin(ROW3_GPIO_Port, ROW3_Pin) == GPIO_PIN_RESET) {
        row |= 0x04; // 3
    } else if (HAL_GPIO_ReadPin(ROW4_GPIO_Port, ROW4_Pin) == GPIO_PIN_RESET) {
        row |= 0x08; // 4
    } else if(HAL_GPIO_ReadPin(ROW5_GPIO_Port, ROW5_Pin) == GPIO_PIN_RESET) {
        row |= 0x10; // 5
    } else if (HAL_GPIO_ReadPin(ROW6_GPIO_Port, ROW6_Pin) == GPIO_PIN_RESET) {
        row |= 0x20; // 6
    } else if (HAL_GPIO_ReadPin(ROW7_GPIO_Port, ROW7_Pin) == GPIO_PIN_RESET) {
        row |= 0x40; // 7
    }
    return row;
}
void key_scan(uint8_t *press_key_list, uint8_t list_size, uint8_t* press_num)
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
    *press_num = 0;
    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
    
    uint8_t rowKey = KeyGetRow();
    if (rowKey != 0) {
        for (uint8_t i = 1; i < 8; i++) {
            if ((rowKey & (1 << (i - 1))) != 0) {
                press_key_list[(*press_num) ++] = keyMap[1][i];
                if (*press_num > list_size) {
                    return;
                }
            }
        }
    }

    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
    
    rowKey = KeyGetRow();
    if (rowKey != 0) {
        for (uint8_t i = 1; i < 8; i++) {
            if ((rowKey & (1 << (i - 1))) != 0) {
                press_key_list[(*press_num) ++] = keyMap[2][i];
                if (*press_num > list_size) {
                    return;
                }
            }
        }
    }

    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
    
    rowKey = KeyGetRow();
    if (rowKey != 0) {
        for (uint8_t i = 1; i < 8; i++) {
            if ((rowKey & (1 << (i - 1))) != 0) {
                press_key_list[(*press_num) ++] = keyMap[3][i];
                if (*press_num > list_size) {
                    return;
                }
            }
        }
    }
    
    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
    
    rowKey = KeyGetRow();
    if (rowKey != 0) {
        for (uint8_t i = 1; i < 8; i++) {
            if ((rowKey & (1 << (i - 1))) != 0) {
                press_key_list[(*press_num) ++] = keyMap[4][i];
                if (*press_num > list_size) {
                    return;
                }
            }
        }
    }

    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_SET);
    
    rowKey = KeyGetRow();
    if (rowKey != 0) {
        for (uint8_t i = 1; i < 8; i++) {
            if ((rowKey & (1 << (i - 1))) != 0) {
                press_key_list[(*press_num) ++] = keyMap[5][i];
                if (*press_num > list_size) {
                    return;
                }
            }
        }
    }

    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL4_GPIO_Port, COL4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL5_GPIO_Port, COL5_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COL6_GPIO_Port, COL6_Pin, GPIO_PIN_RESET);
    
    rowKey = KeyGetRow();
    if (rowKey != 0) {
        for (uint8_t i = 1; i < 8; i++) {
            if ((rowKey & (1 << (i - 1))) != 0) {
                press_key_list[(*press_num) ++] = keyMap[6][i];
                if (*press_num > list_size) {
                    return;
                }
            }
        }
    }    
}
