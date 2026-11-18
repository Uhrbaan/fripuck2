#include "speaker.h"
#include "main.h"

#define SINUS_BUFFER_SIZE 360
#define TIM6_CLOCK_HZ 84000000UL
#define BASE_AMPLITUDE 256
#define MAX_SINE_AMP 900

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;

static uint8_t dac_is_playing = 0;

static const uint16_t sinus_buffer[SINUS_BUFFER_SIZE] = {
    2047, 2052, 2056, 2061, 2065, 2070, 2074, 2079, 2083, 2087, 2092, 2096, 2101, 2105, 2109, 2114, 2118, 2122, 2126,
    2131, 2135, 2139, 2143, 2147, 2151, 2156, 2160, 2164, 2168, 2171, 2175, 2179, 2183, 2187, 2190, 2194, 2198, 2201,
    2205, 2208, 2212, 2215, 2219, 2222, 2225, 2228, 2231, 2235, 2238, 2240, 2243, 2246, 2249, 2252, 2254, 2257, 2260,
    2262, 2264, 2267, 2269, 2271, 2273, 2275, 2277, 2279, 2281, 2283, 2285, 2286, 2288, 2289, 2291, 2292, 2293, 2295,
    2296, 2297, 2298, 2299, 2299, 2300, 2301, 2301, 2302, 2302, 2303, 2303, 2303, 2303, 2303, 2303, 2303, 2303, 2303,
    2302, 2302, 2301, 2301, 2300, 2299, 2299, 2298, 2297, 2296, 2295, 2293, 2292, 2291, 2289, 2288, 2286, 2285, 2283,
    2281, 2279, 2277, 2275, 2273, 2271, 2269, 2267, 2264, 2262, 2260, 2257, 2254, 2252, 2249, 2246, 2243, 2240, 2238,
    2235, 2231, 2228, 2225, 2222, 2219, 2215, 2212, 2208, 2205, 2201, 2198, 2194, 2190, 2187, 2183, 2179, 2175, 2171,
    2168, 2164, 2160, 2156, 2151, 2147, 2143, 2139, 2135, 2131, 2126, 2122, 2118, 2114, 2109, 2105, 2101, 2096, 2092,
    2087, 2083, 2079, 2074, 2070, 2065, 2061, 2056, 2052, 2047, 2043, 2039, 2034, 2030, 2025, 2021, 2016, 2012, 2007,
    2003, 1999, 1994, 1990, 1986, 1981, 1977, 1973, 1968, 1964, 1960, 1956, 1952, 1948, 1943, 1939, 1935, 1931, 1927,
    1923, 1920, 1916, 1912, 1908, 1904, 1901, 1897, 1894, 1890, 1886, 1883, 1880, 1876, 1873, 1870, 1867, 1863, 1860,
    1857, 1854, 1851, 1849, 1846, 1843, 1840, 1838, 1835, 1833, 1831, 1828, 1826, 1824, 1822, 1820, 1818, 1816, 1814,
    1812, 1810, 1809, 1807, 1806, 1804, 1803, 1802, 1800, 1799, 1798, 1797, 1796, 1796, 1795, 1794, 1794, 1793, 1793,
    1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1792, 1793, 1793, 1794, 1794, 1795, 1796, 1796, 1797, 1798, 1799,
    1800, 1802, 1803, 1804, 1806, 1807, 1809, 1810, 1812, 1814, 1816, 1818, 1820, 1822, 1824, 1826, 1828, 1831, 1833,
    1835, 1838, 1840, 1843, 1846, 1849, 1851, 1854, 1857, 1860, 1863, 1867, 1870, 1873, 1876, 1880, 1883, 1886, 1890,
    1894, 1897, 1901, 1904, 1908, 1912, 1916, 1920, 1923, 1927, 1931, 1935, 1939, 1943, 1948, 1952, 1956, 1960, 1964,
    1968, 1973, 1977, 1981, 1986, 1990, 1994, 1999, 2003, 2007, 2012, 2016, 2021, 2025, 2030, 2034, 2039, 2043};

static uint16_t active_buffer[SINUS_BUFFER_SIZE];
static uint8_t current_volume = 10;  // Volume par défaut : 10/10
static speaker_waveform_t current_wave = WAVE_SINE;

static const uint16_t wave_gain_table[] = {
    [WAVE_SINE] = 100, [WAVE_TRIANGLE] = 125, [WAVE_SQUARE] = 12, [WAVE_SAWTOOTH] = 16};

static void update_active_buffer(void) {
    if (current_volume == 0) {
        for (uint16_t i = 0; i < SINUS_BUFFER_SIZE; i++) {
            active_buffer[i] = 2048;
        }
        return;
    }

    // Courbe quadratique : progression perçue linéaire de V1 (1/225) à V15 (225/225)
    int32_t vol_curve = (int32_t)current_volume * (int32_t)current_volume;  // 1 .. 225
    int32_t wave_gain_pct = wave_gain_table[current_wave];

    // Amplitude crête cible pour l'instrument courant
    int32_t amp = (MAX_SINE_AMP * vol_curve * wave_gain_pct) / (225 * 100);
    uint16_t quarter = SINUS_BUFFER_SIZE / 4;

    for (uint16_t i = 0; i < SINUS_BUFFER_SIZE; i++) {
        int32_t offset = 0;

        switch (current_wave) {
            case WAVE_SINE: {
                // sinus_buffer a une amplitude de base de 256 autour de 2048
                int32_t base_sin = (int32_t)sinus_buffer[i] - 2048;
                offset = (base_sin * amp) / 256;
                break;
            }

            case WAVE_TRIANGLE:
                if (i < quarter) {
                    offset = (amp * i) / quarter;
                } else if (i < 3 * quarter) {
                    offset = amp - (amp * (i - quarter)) / quarter;
                } else {
                    offset = -amp + (amp * (i - 3 * quarter)) / quarter;
                }
                break;

            case WAVE_SQUARE:
                offset = (i < SINUS_BUFFER_SIZE / 2) ? amp : -amp;
                break;

            case WAVE_SAWTOOTH:
                if (i < SINUS_BUFFER_SIZE / 2) {
                    offset = (2 * amp * i) / SINUS_BUFFER_SIZE;
                } else {
                    offset = -2 * amp + (2 * amp * i) / SINUS_BUFFER_SIZE;
                }
                break;

            default:
                break;
        }

        int32_t sample = 2048 + offset;
        if (sample < 0) sample = 0;
        if (sample > 4095) sample = 4095;
        active_buffer[i] = (uint16_t)sample;
    }
}

void speaker_set_volume(uint8_t volume) {
    if (volume > 15) volume = 15;
    current_volume = volume;
    update_active_buffer();
}

void speaker_set_waveform(speaker_waveform_t wave) {
    if (wave > WAVE_SAWTOOTH) wave = WAVE_SINE;
    current_wave = wave;
    update_active_buffer();
}

void speaker_init(void) {
    HAL_GPIO_WritePin(AUDIO_PWR_GPIO_Port, AUDIO_PWR_Pin, GPIO_PIN_SET);  // Off at boot
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
}

void speaker_mute(void) {
    if (dac_is_playing) {
        // Attendre que l'index DMA repasse près de 0 (zéro analogique) avant de couper (max ~3 ms)
        uint32_t timeout = 50000;
        while (__HAL_DMA_GET_COUNTER(hdac.DMA_Handle2) < (SINUS_BUFFER_SIZE - 10) && --timeout) {
            __NOP();
        }
        HAL_TIM_Base_Stop(&htim6);
        HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
        dac_is_playing = 0;
    }
}

void speaker_stop(void) {
    speaker_mute();
    HAL_GPIO_WritePin(AUDIO_PWR_GPIO_Port, AUDIO_PWR_Pin, GPIO_PIN_SET);  // Power off amp
}

void speaker_play_frequency(uint16_t freq) {
    if (freq == 0) {
        speaker_mute();
        return;
    }

    uint32_t arr = (TIM6_CLOCK_HZ / ((uint32_t)freq * SINUS_BUFFER_SIZE));
    if (arr > 0) arr -= 1;
    if (arr > 0xFFFF) arr = 0xFFFF;

    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);

    if (!dac_is_playing) {
        dac_is_playing = 1;
        HAL_GPIO_WritePin(AUDIO_PWR_GPIO_Port, AUDIO_PWR_Pin, GPIO_PIN_RESET);  // Power ON amp (active-low)
        HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)active_buffer, SINUS_BUFFER_SIZE, DAC_ALIGN_12B_R);
        HAL_TIM_Base_Start(&htim6);
    } else {
        if (__HAL_TIM_GET_COUNTER(&htim6) >= arr) {
            __HAL_TIM_SET_COUNTER(&htim6, 0);
        }
    }
}

uint8_t speaker_is_playing(void) { return dac_is_playing; }