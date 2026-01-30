# Чтение и кольцевая запись EEPROM по I2C для STM32

Ниже приведён практический подход для внешней I2C‑EEPROM (типично 24xx/AT24Cxx) с STM32. Он учитывает страничную запись, задержку цикла записи и организует кольцевой буфер (ring buffer) с устойчивостью к обрыву питания.

## Базовые особенности I2C‑EEPROM

- **Страничная запись (page write).** Запись допустима только в пределах страницы фиксированного размера (обычно 8–64 байт). Если начать запись с адреса ближе к концу страницы, данные **перепрыгнут на начало страницы**, поэтому данные нужно делить по границам страниц.
- **Время цикла записи (write cycle).** После записи EEPROM занят несколько миллисекунд и отвечает NACK. Стандартный подход — **ACK polling**: периодически опрашивать устройство, пока оно снова не подтвердит адрес.
- **Адресация.** Большинство 24xx используют 7‑битный I2C‑адрес с битами A0..A2, а адрес памяти передаётся как 1 или 2 байта (зависит от объёма).

## Ринг‑буфер (кольцевая запись)

Цель — сохранять последовательные записи фиксированного размера, перезаписывая самые старые. Надёжный вариант:

1. **Структура записи**: `seq` (счётчик), `len` (или фиксированный размер), `payload`, `crc`.
2. **Запись**:
   - вычислить позицию `write_ptr` (адрес) и записать запись;
   - увеличить `seq` и сдвинуть указатель на размер записи (с переходом в начало области).
3. **Чтение (по последней записи)**:
   - сканировать область, находить запись с максимальным `seq` и корректным `crc`;
   - вычислить `read_ptr`.

Такой подход сохраняет **последнюю валидную запись** даже при обрыве питания, потому что каждая запись самодостаточна и проверяется `crc`.

## Пример конфигурации

- Память: 24LC256 (32 КБ), адресация 16‑бит.
- Размер страницы: 64 байта.
- Область под лог: например, 0x0000–0x3FFF.
- Размер записи: 32 байта (влезает в страницу, либо делится на 2 страницы при необходимости).

## Пример кода (STM32 HAL, I2C)

> Пример не привязан к конкретной серии STM32, но рассчитан на HAL (`HAL_I2C_Mem_Read/Write`).

```c
#include "stm32f1xx_hal.h" // замените на свою серию
#include <string.h>
#include <stdint.h>

#define EEPROM_I2C_ADDR  (0x50 << 1) // 7-bit адрес EEPROM, сдвинутый для HAL
#define EEPROM_PAGE_SIZE 64
#define EEPROM_SIZE      (32 * 1024)

#define LOG_START 0x0000
#define LOG_END   0x3FFF
#define LOG_SIZE  (LOG_END - LOG_START + 1)

typedef struct {
    uint32_t seq;
    uint16_t len;
    uint8_t  data[24];
    uint16_t crc;
} log_record_t;

static uint16_t crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

static HAL_StatusTypeDef eeprom_wait_ready(I2C_HandleTypeDef *hi2c) {
    for (uint32_t i = 0; i < 1000; i++) {
        if (HAL_I2C_IsDeviceReady(hi2c, EEPROM_I2C_ADDR, 1, 5) == HAL_OK) {
            return HAL_OK;
        }
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef eeprom_write_page(
    I2C_HandleTypeDef *hi2c,
    uint16_t mem_addr,
    const uint8_t *data,
    uint16_t len
) {
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(
        hi2c,
        EEPROM_I2C_ADDR,
        mem_addr,
        I2C_MEMADD_SIZE_16BIT,
        (uint8_t *)data,
        len,
        HAL_MAX_DELAY
    );
    if (st != HAL_OK) {
        return st;
    }
    return eeprom_wait_ready(hi2c);
}

static HAL_StatusTypeDef eeprom_write(
    I2C_HandleTypeDef *hi2c,
    uint16_t mem_addr,
    const uint8_t *data,
    uint16_t len
) {
    while (len > 0) {
        uint16_t page_off = mem_addr % EEPROM_PAGE_SIZE;
        uint16_t chunk = EEPROM_PAGE_SIZE - page_off;
        if (chunk > len) {
            chunk = len;
        }
        HAL_StatusTypeDef st = eeprom_write_page(hi2c, mem_addr, data, chunk);
        if (st != HAL_OK) {
            return st;
        }
        mem_addr += chunk;
        data += chunk;
        len -= chunk;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef eeprom_read(
    I2C_HandleTypeDef *hi2c,
    uint16_t mem_addr,
    uint8_t *data,
    uint16_t len
) {
    return HAL_I2C_Mem_Read(
        hi2c,
        EEPROM_I2C_ADDR,
        mem_addr,
        I2C_MEMADD_SIZE_16BIT,
        data,
        len,
        HAL_MAX_DELAY
    );
}

static uint16_t ring_next(uint16_t addr, uint16_t rec_size) {
    uint16_t next = addr + rec_size;
    if (next + rec_size - 1 > LOG_END) {
        return LOG_START;
    }
    return next;
}

HAL_StatusTypeDef ring_write(
    I2C_HandleTypeDef *hi2c,
    uint16_t *write_ptr,
    uint32_t *seq,
    const uint8_t *payload,
    uint16_t payload_len
) {
    log_record_t rec;
    memset(&rec, 0xFF, sizeof(rec));

    rec.seq = *seq;
    rec.len = payload_len;
    memcpy(rec.data, payload, sizeof(rec.data));
    rec.crc = crc16((uint8_t *)&rec, sizeof(rec) - sizeof(rec.crc));

    HAL_StatusTypeDef st = eeprom_write(
        hi2c,
        *write_ptr,
        (uint8_t *)&rec,
        sizeof(rec)
    );
    if (st != HAL_OK) {
        return st;
    }

    *seq += 1;
    *write_ptr = ring_next(*write_ptr, sizeof(rec));
    return HAL_OK;
}
```

## Поиск последней записи (после включения)

- Просканировать всю область `LOG_START..LOG_END` шагом `sizeof(log_record_t)`.
- Для каждой записи проверить `crc`.
- Запомнить запись с максимальным `seq` — это последняя валидная запись.

Если область пуста (все `0xFF`) — инициализировать `seq = 0`, `write_ptr = LOG_START`.

## Практические советы

- **Соблюдайте границы страниц.** Иначе данные «переедут» в начало страницы.
- **Учитывайте износ** (обычно 1e5 циклов). Кольцевая запись даёт равномерное распределение.
- **Делайте запись атомарной.** Храните `crc` и `seq`, чтобы игнорировать частично записанные записи.
- **Проверяйте даташит EEPROM**, особенно адресацию (8‑бит/16‑бит) и размер страницы.
