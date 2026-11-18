#include "mml_player.h"
#include "speaker.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "opcodes.h"

#define MML_QUEUE_LENGTH 4
#define MML_TASK_STACK_SIZE 256
#define MML_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define MAX_LOOP_DEPTH 2

typedef struct {
    const char* token;
    uint8_t len;
    int8_t semitone;  // 0=C(do) .. 11=B(si), -1=Rest
    uint8_t is_solfege;
} note_token_t;

// Ordered longest-first so "sol" matches before any single-letter prefix
static const note_token_t note_table[] = {
    {"sol", 3, 7, 1}, {"do", 2, 0, 1}, {"re", 2, 2, 1}, {"mi", 2, 4, 1}, {"fa", 2, 5, 1}, {"la", 2, 9, 1},
    {"si", 2, 11, 1}, {"c", 1, 0, 0},  {"d", 1, 2, 0},  {"e", 1, 4, 0},  {"f", 1, 5, 0},  {"g", 1, 7, 0},
    {"a", 1, 9, 0},   {"b", 1, 11, 0}, {"r", 1, -1, 0}, {"p", 1, -1, 0},
};

// Octave 4 base frequencies: C, C#, D, D#, E, F, F#, G, G#, A, A#, B
static const uint16_t base_freq_o4[12] = {262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494};

static QueueHandle_t xMmlQueue = NULL;
static TaskHandle_t xMmlTaskHandle = NULL;
static volatile mml_notation_t current_notation = MML_NOTATION_AUTO;
static volatile uint8_t abort_flag = 0;

static int match_note_token(const char** ptr, int8_t* semitone_out) {
    mml_notation_t mode = current_notation;
    size_t count = sizeof(note_table) / sizeof(note_table[0]);

    for (size_t i = 0; i < count; i++) {
        const note_token_t* nt = &note_table[i];

        if (nt->semitone >= 0) {
            if (mode == MML_NOTATION_ENGLISH && nt->is_solfege) continue;
            if (mode == MML_NOTATION_SOLFEGE && !nt->is_solfege) continue;
        }

        if (strncmp(*ptr, nt->token, nt->len) == 0) {
            char next = (*ptr)[nt->len];
            if (nt->len == 1 && islower((unsigned char)next)) {
                continue;
            }
            *ptr += nt->len;
            *semitone_out = nt->semitone;
            return 1;
        }
    }
    return 0;
}

static uint32_t parse_uint(const char** ptr, uint32_t default_val) {
    if (!isdigit((unsigned char)**ptr)) {
        return default_val;
    }
    char* end;
    uint32_t val = strtoul(*ptr, &end, 10);
    *ptr = end;
    return val;
}

static void play_parsed_note(const char** mml_ptr, int8_t semitone, uint16_t bpm, uint8_t octave, uint8_t default_len,
                             uint8_t quantize) {
    const char* mml = *mml_ptr;

    // 1. Accidentals (#, +, -)
    if (semitone >= 0) {
        switch (*mml) {
            case '#':
            case '+':
                semitone++;
                mml++;
                break;
            case '-':
                semitone--;
                mml++;
                break;
            default:
                break;
        }
    }

    // 2. Note length & dots
    uint8_t len = (uint8_t)parse_uint(&mml, default_len);
    if (len == 0) len = default_len;

    uint32_t whole_ms = (60000UL * 4) / bpm;
    uint32_t duration_ms = whole_ms / len;

    uint32_t dot_add = duration_ms / 2;
    while (*mml == '.') {
        duration_ms += dot_add;
        dot_add /= 2;
        mml++;
    }

    // 3. Tie ('&')
    uint8_t tied = 0;
    while (*mml == ' ') mml++;
    if (*mml == '&') {
        tied = 1;
        mml++;
    }

    *mml_ptr = mml;

    // 4. Compute frequency
    uint16_t freq = 0;
    if (semitone >= 0) {
        int8_t adj_oct = (int8_t)octave;
        if (semitone >= 12) {
            semitone -= 12;
            adj_oct++;
        } else if (semitone < 0) {
            semitone += 12;
            adj_oct--;
        }

        freq = base_freq_o4[semitone];
        if (adj_oct > 4)
            freq <<= (adj_oct - 4);
        else if (adj_oct < 4)
            freq >>= (4 - adj_oct);
    }

    // 5. Play tone + articulation gap
    uint32_t tone_ms = tied ? duration_ms : (duration_ms * quantize) / 8;
    uint32_t gap_ms = duration_ms - tone_ms;

    if (freq > 0) {
        speaker_play_frequency(freq);
    } else {
        speaker_mute();
    }

    if (tone_ms > 0) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(tone_ms)) > 0 || abort_flag) return;
    }

    if (gap_ms > 0) {
        speaker_mute();
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(gap_ms)) > 0 || abort_flag) return;
    }
}

static void execute_mml(const char* mml) {
    uint16_t bpm = 120;
    uint8_t octave = 4;
    uint8_t default_len = 4;
    uint8_t quantize = 7;

    const char* loop_starts[MAX_LOOP_DEPTH] = {NULL};
    uint8_t loop_counts[MAX_LOOP_DEPTH] = {0};
    int8_t loop_sp = -1;

    while (*mml && !abort_flag) {
        char c = *mml;

        switch (c) {
            case ' ':
            case '\t':
            case '\r':
            case '\n':
                mml++;
                break;

            case 'T': {
                mml++;
                uint32_t val = parse_uint(&mml, 120);
                if (val >= 30 && val <= 600) bpm = (uint16_t)val;
                break;
            }

            case 'O': {
                mml++;
                uint32_t val = parse_uint(&mml, 4);
                if (val >= 1 && val <= 7) octave = (uint8_t)val;
                break;
            }

            case 'L': {
                mml++;
                uint32_t val = parse_uint(&mml, 4);
                if (val >= 1 && val <= 64) default_len = (uint8_t)val;
                break;
            }

            case 'Q': {
                mml++;
                uint32_t val = parse_uint(&mml, 7);
                if (val >= 1 && val <= 8) quantize = (uint8_t)val;
                break;
            }

            case 'V': {
                mml++;
                uint32_t val = parse_uint(&mml, 10);
                if (val <= 15) {
                    speaker_set_volume((uint8_t)val);
                }
                break;
            }

            case 'I': {
                mml++;
                uint32_t val = parse_uint(&mml, 0);
                if (val <= 3) {
                    speaker_set_waveform((speaker_waveform_t)val);
                }
                break;
            }

            case '>':
                mml++;
                if (octave < 7) octave++;
                break;

            case '<':
                mml++;
                if (octave > 1) octave--;
                break;

            case '[':
                mml++;
                if (loop_sp + 1 < MAX_LOOP_DEPTH) {
                    loop_sp++;
                    loop_starts[loop_sp] = mml;
                    loop_counts[loop_sp] = 0;
                }
                break;

            case ']': {
                mml++;
                uint8_t repeats = (uint8_t)parse_uint(&mml, 2);
                if (loop_sp >= 0) {
                    if (loop_counts[loop_sp] == 0) {
                        if (repeats > 1) {
                            loop_counts[loop_sp] = repeats - 1;
                            mml = loop_starts[loop_sp];
                        } else {
                            loop_sp--;
                        }
                    } else if (--loop_counts[loop_sp] > 0) {
                        mml = loop_starts[loop_sp];
                    } else {
                        loop_sp--;
                    }
                }
                break;
            }

            default:
                if (islower((unsigned char)c)) {
                    int8_t semitone = -1;
                    if (match_note_token(&mml, &semitone)) {
                        play_parsed_note(&mml, semitone, bpm, octave, default_len, quantize);
                    } else {
                        mml++;
                    }
                } else {
                    mml++;  // Skip unknown symbol
                }
                break;
        }
    }

    speaker_stop();
}

static void mml_player_task(void* pvParameters) {
    (void)pvParameters;
    const char* song_ptr = NULL;

    for (;;) {
        if (xQueueReceive(xMmlQueue, &song_ptr, portMAX_DELAY) == pdTRUE) {
            if (song_ptr != NULL) {
                abort_flag = 0;
                ulTaskNotifyTake(pdTRUE, 0);
                execute_mml(song_ptr);
            }
        }
    }
}

void mml_player_init(void) {
    speaker_init();
    xMmlQueue = xQueueCreate(MML_QUEUE_LENGTH, sizeof(const char*));
    xTaskCreate(mml_player_task, "MML_Player", MML_TASK_STACK_SIZE, NULL, MML_TASK_PRIORITY, &xMmlTaskHandle);
}

void mml_set_notation(mml_notation_t notation) { current_notation = notation; }

#define MML_POOL_SIZE 4

static char mml_pool[MML_POOL_SIZE][MML_MAX_STR_LEN];
static uint8_t pool_write_idx = 0;

void mml_play(const char* mml_string, mml_play_mode_t mode) {
    if (mml_string == NULL || xMmlQueue == NULL) return;

    if (mode == MML_PLAY_IF_FREE) {
        if (uxQueueMessagesWaiting(xMmlQueue) > 0 || speaker_is_playing()) {
            return;
        }
    }

    // Copie sécurisée dans le prochain slot libre du pool statique
    char* slot = mml_pool[pool_write_idx];
    strncpy(slot, mml_string, MML_MAX_STR_LEN - 1);
    slot[MML_MAX_STR_LEN - 1] = '\0';
    pool_write_idx = (pool_write_idx + 1) % MML_POOL_SIZE;

    if (mode == MML_FORCE_REPLACE) {
        xQueueReset(xMmlQueue);
        abort_flag = 1;
        xQueueSend(xMmlQueue, &slot, 0);
        xTaskNotifyGive(xMmlTaskHandle);
    } else {
        xQueueSend(xMmlQueue, &slot, (mode == MML_QUEUE_NEXT) ? portMAX_DELAY : 0);
    }
}

void mml_stop(void) {
    if (xMmlQueue == NULL) return;
    xQueueReset(xMmlQueue);
    abort_flag = 1;
    xTaskNotifyGive(xMmlTaskHandle);
}