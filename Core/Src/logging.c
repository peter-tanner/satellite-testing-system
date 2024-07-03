#include "logging.h"

static void uint32_to_string(uint32_t number, char str[])
{
    int i = 0;
    char temp[12];

    if (number == 0)
    {
        str[i++] = '0';
        str[i++] = '\n';
        str[i] = '\0';
        return;
    }

    while (number > 0)
    {
        temp[i++] = (number % 10) + '0';
        number /= 10;
    }

    int j;
    for (j = 0; j < i; j++)
    {
        str[j] = temp[i - j - 1];
    }
    str[j++] = '\n';
    str[j] = '\0';
}

const uint8_t TEST_TEXT[] =
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~ \n"
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~ \n"
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~ \n"
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~ \n"
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~ \n"
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~ \n"
    "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~ \n";

void StartMMCWriteTask(void const *argument)
{
    FRESULT res;                      /* FatFs function common result code */
    uint32_t byteswritten, bytesread; /* File write/read counts */
    uint32_t counter = 0;
    uint8_t counter_str[13];

// #define FORMAT_VOLUME
#ifdef FORMAT_VOLUME
    {
        uint8_t rtext[_MAX_SS]; /* File read buffer */
        FRESULT res;
        res = f_mkfs((TCHAR const *)USERPath, FM_ANY, 0, rtext, sizeof(rtext));
        if (res != FR_OK)
            Error_Handler();
    }
#endif

    res = f_mount(&USERFatFS, (TCHAR const *)USERPath, 0);
    if (res != FR_OK)
        Error_Handler();

    // Open file for writing (Create)
    // FILE *NAME* (NOT INCLUDING DIRS) MUST BE MAX 9 CHARS
    res = f_open(&USERFile, "ASCII_CHARACTERS.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
        Error_Handler();
    // Write to the text file
    res = f_write(&USERFile, TEST_TEXT, strlen((char *)TEST_TEXT), (void *)&byteswritten);
    if ((byteswritten == 0) || (res != FR_OK))
        Error_Handler();
    f_close(&USERFile);

    res = f_open(&USERFile, "TEST_LOG.TXT", FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK)
        Error_Handler();
    uint8_t intro_txt[] = "--- NEW SESSION ---\n";
    res = f_write(&USERFile, intro_txt, strlen(intro_txt), (void *)&byteswritten);
    if ((byteswritten == 0) || (res != FR_OK))
        Error_Handler();
    f_close(&USERFile);

    for (;;)
    {
        // Write to the text file
        res = f_open(&USERFile, "TEST_LOG.TXT", FA_OPEN_APPEND | FA_WRITE);
        if (res != FR_OK)
            Error_Handler();

        if (f_lseek(&USERFile, f_size(&USERFile)) != FR_OK)
            Error_Handler();

        uint32_to_string(HAL_GetTick(), counter_str);

        res = f_write(&USERFile, counter_str, strlen(counter_str), (void *)&byteswritten);
        if ((byteswritten == 0) || (res != FR_OK))
            ; // Error_Handler();// TODO: Add logging event or something

        f_close(&USERFile);

        // Delay for 1 second
        osDelay(1000);

        counter++;
    }
}