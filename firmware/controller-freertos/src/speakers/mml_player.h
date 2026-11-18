/**
 * @file    mml_player.h
 * @brief   Lecteur de mélodies MML (Music Macro Language) asynchrone sous FreeRTOS.
 *
 * @details
 * **Fonctionnement :**
 * Ce module exécute une tâche FreeRTOS dédiée qui lit des partitions sous forme de
 * chaînes ASCII depuis une file d'attente (Queue). Le son est généré sans charge CPU
 * via le DAC (PA5) piloté en DMA circulaire par le timer TIM6, tandis que l'amplificateur
 * audio est activé à la demande via PD7 (actif à l'état bas). Les délais entre notes
 * utilisent `ulTaskNotifyTake()`, permettant d'interrompre instantanément un morceau.
 *
 * **Syntaxe MML (Notes en minuscules, Commandes en MAJUSCULES) :**
 *
 * | Syntaxe       | Description                                      | Exemple / Défaut         |
 * |---------------|--------------------------------------------------|--------------------------|
 * | `a` - `g`     | Notes anglo-saxonnes (la, si, do, ré, mi, fa, sol)| `c`, `f#`, `b-`          |
 * | `do`..`si`    | Notes en solfège (`do`, `re`, `mi`, `fa`, `sol`, `la`, `si`) | `sol4`, `do#8` |
 * | `r` ou `p`    | Silence (Rest / Pause)                           | `r4`, `p8`               |
 * | `#` / `+` / `-`| Altérations : dièse (`#` ou `+`), bémol (`-`)   | `c#`, `sol+`, `mi-`      |
 * | `<n>`         | Durée de la note (`1`=ronde, `4`=noire, `8`=croche, `16`=double) | `do8` (défaut: `L`) |
 * | `.`           | Note pointée (allonge la durée de 50%, cumulable)| `la4.`, `sol2..`         |
 * | `&`           | Liaison (tie) : supprime le silence d'articulation vers la note suivante | `do4 & do8` |
 * | `T<n>`        | Tempo en battements par minute (30 à 400 BPM)    | `T140` (défaut: `T120`)  |
 * | `O<n>`        | Sélection de l'octave (`1` à `7`)                | `O5` (défaut: `O4`)      |
 * | `>` / `<`     | Monte (`>`) ou descend (`<`) d'une octave        | `>do <sol`               |
 * | `L<n>`        | Durée par défaut des notes sans chiffre (`1` à `64`) | `L8` (défaut: `L4`)  |
 * | `V<n>`        | Volume (`0`=muet, `10`=max nominal, `11`-`15`=boost) | `V12` (défaut: `V10`) |
 * | `Q<n>`        | Articulation (`1` à `8` : `8`=legato, `7`=normal, `4`=staccato) | `Q6` (défaut: `Q7`) |
 * | `[ ... ]<n>`  | Répète le bloc `<n>` fois (2 niveaux d'imbrication max) | `[do re mi]2` (défaut: 2)|
 * | `I<n>`        | Instrument / Forme d'onde (`0`=Sinus, `1`=Triangle, `2`=Carré, `3`=Dent de scie) | `I2` (défaut:
 * `I0`) |
 *
 * @example
 * // Exemple d'utilisation :
 * mml_player_init();
 * mml_play("T150 V12 O5 L8 [ do re mi4 ]2 sol2", MML_FORCE_REPLACE);
 */

#pragma once

#include <stdint.h>

typedef enum {
    MML_PLAY_IF_FREE,  /**< Joue uniquement si aucune mélodie n'est en cours */
    MML_FORCE_REPLACE, /**< Interrompt la mélodie actuelle et joue immédiatement */
    MML_QUEUE_NEXT     /**< Ajoute la mélodie à la file d'attente */
} mml_play_mode_t;

typedef enum {
    MML_NOTATION_AUTO,    /**< Accepte simultanément `a-g` et `do-si` */
    MML_NOTATION_ENGLISH, /**< Uniquement `a`, `b`, `c`, `d`, `e`, `f`, `g` */
    MML_NOTATION_SOLFEGE  /**< Uniquement `do`, `re`, `mi`, `fa`, `sol`, `la`, `si` */
} mml_notation_t;

void mml_player_init(void);
void mml_set_notation(mml_notation_t notation);
void mml_play(const char* mml_string, mml_play_mode_t mode);
void mml_stop(void);
