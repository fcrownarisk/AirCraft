/*=== 12 QUARK TYPES — KERNEL IN C ===
 * Flavors: up, down, charm, strange, top, bottom (×6)
 * Colors: red, green, blue (×3)
 * Antimatter: matter, antimatter (×2)
 * Total: 6×3×2 = 36 combinations → 12 distinct type categories
 * 
 * Compressed representation — zero entropy kernel
 */

#include <stdint.h"
#include <stdbool.h"

/*--- 1. ENUM DOMAIN (full factor space) ---*/
typedef enum { UP=0, DOWN=1, CHARM=2, STRANGE=3, TOP=4, BOTTOM=5 } Flavor;
typedef enum { RED=0, GREEN=1, BLUE=2 } Color;
typedef enum { MATTER=0, ANTIMATTER=1 } Parity;
typedef enum { QUARK=0, ANTIQUARK=1, DIQUARK=2, TRIPLET=3 } Composite;

/*--- 2. PROPERTY TABLES (ROM, zero runtime entropy) ---*/
static const double MASS_MEV[6]   = {2.2, 4.7, 1275.0, 95.0, 173070.0, 4180.0};
static const double CHARGE_E[6]   = {2.0/3.0, -1.0/3.0, 2.0/3.0, -1.0/3.0, 2.0/3.0, -1.0/3.0};
static const double SPIN[6]       = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
static const int    BARYON[6]     = {1, 1, 1, 1, 1, 1};
static const char*  NAME[6]       = {"u","d","c","s","t","b"};
static const char*  COLOR_STR[3]  = {"r","g","b"};

/*--- 3. COMPACT TYPE (single uint16_t encodes all dimensions) ---*/
typedef uint16_t QuarkID;  /* bits: [15:12]reserved [11:9]composite [8]parity [7:5]color [4:0]flavor */

#define QID(f,c,p,o) (((QuarkID)(o)<<9)|((QuarkID)(p)<<8)|((QuarkID)(c)<<5)|((QuarkID)(f)))
#define Q_FLAVOR(id)  ((id)&0x1F)
#define Q_COLOR(id)   (((id)>>5)&0x7)
#define Q_PARITY(id)  (((id)>>8)&0x1)
#define Q_COMP(id)    (((id)>>9)&0x7)

/*--- 4. 12 CANONICAL QUARK TYPES (all single-quark matter states) ---*/
static const QuarkID QUARK_UP_RED      = QID(UP,     RED,   MATTER,     QUARK);
static const QuarkID QUARK_UP_GREEN    = QID(UP,     GREEN, MATTER,     QUARK);
static const QuarkID QUARK_UP_BLUE     = QID(UP,     BLUE,  MATTER,     QUARK);
static const QuarkID QUARK_DOWN_RED    = QID(DOWN,   RED,   MATTER,     QUARK);
static const QuarkID QUARK_DOWN_GREEN  = QID(DOWN,   GREEN, MATTER,     QUARK);
static const QuarkID QUARK_DOWN_BLUE   = QID(DOWN,   BLUE,  MATTER,     QUARK);
static const QuarkID QUARK_CHARM_RED   = QID(CHARM,  RED,   MATTER,     QUARK);
static const QuarkID QUARK_STRANGE_GREEN=QID(STRANGE,GREEN, MATTER,     QUARK);
static const QuarkID QUARK_TOP_RED     = QID(TOP,    RED,   MATTER,     QUARK);
static const QuarkID QUARK_BOTTOM_BLUE = QID(BOTTOM, BLUE,  MATTER,     QUARK);
static const QuarkID ANTIQUARK_UP_RED  = QID(UP,     RED,   ANTIMATTER, ANTIQUARK);
static const QuarkID ANTIQUARK_DOWN_GREEN=QID(DOWN, GREEN, ANTIMATTER, ANTIQUARK);

/*--- 5. KERNEL OPERATIONS (stateless, pure functions) ---*/
static inline double quark_mass(QuarkID q)    { return MASS_MEV[Q_FLAVOR(q)]; }
static inline double quark_charge(QuarkID q)  { return Q_PARITY(q)==MATTER ? CHARGE_E[Q_FLAVOR(q)] : -CHARGE_E[Q_FLAVOR(q)]; }
static inline bool   quark_is_heavy(QuarkID q){ return Q_FLAVOR(q) >= CHARM; }
static inline QuarkID quark_antiparticle(QuarkID q) { return q ^ (1<<8); }  /* flip parity bit */
static inline QuarkID quark_color_swap(QuarkID q, Color c) { return (q & ~(0x7<<5)) | ((QuarkID)c<<5); }

typedef struct { QuarkID q[3]; } Baryon;  /* proton = uud, neutron = udd */

static inline bool baryon_is_colorless(Baryon b) {
    return (Q_COLOR(b.q[0]) != Q_COLOR(b.q[1])) && 
           (Q_COLOR(b.q[1]) != Q_COLOR(b.q[2])) && 
           (Q_COLOR(b.q[0]) != Q_COLOR(b.q[2]));
}

/*--- 6. PRINT TABLE (12 types, zero branching) ---*/
#include <stdio.h>
int main(void) {
    static const QuarkID list[12] = {
        QUARK_UP_RED, QUARK_UP_GREEN, QUARK_UP_BLUE,
        QUARK_DOWN_RED, QUARK_DOWN_GREEN, QUARK_DOWN_BLUE,
        QUARK_CHARM_RED, QUARK_STRANGE_GREEN,
        QUARK_TOP_RED, QUARK_BOTTOM_BLUE,
        ANTIQUARK_UP_RED, ANTIQUARK_DOWN_GREEN
    };
    
    for (int i=0; i<12; i++) {
        QuarkID q = list[i];
        printf("%s%s_%s  mass=%.1f MeV  Q=%.3fe  spin=%.1f\n",
               Q_PARITY(q)?"anti":"",
               NAME[Q_FLAVOR(q)],
               COLOR_STR[Q_COLOR(q)],
               quark_mass(q),
               quark_charge(q),
               SPIN[Q_FLAVOR(q)]);
    }
    return 0;
}