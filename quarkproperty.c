/*=== 12 QUARK TYPES — COMPLETE KERNEL (SPIN, CHARGE, MASS) ===
 * 
 * Standard Model quark sector: 6 flavors × 3 colors = 18 quarks
 * + 18 antiquarks = 36 fundamental particles
 * 12 shown here as canonical representatives covering all properties
 *
 * Properties encoded:
 *   SPIN:   intrinsic angular momentum (ℏ units)
 *   CHARGE: electric charge (e units)  
 *   MASS:   invariant mass (MeV/c², PDG 2024)
 *   COLOR:  SU(3) gauge charge (r,g,b)
 *   FLAVOR: weak isospin eigenstate label
 *   BARYON: baryon number (+1/3 per quark)
 *   ISOSPIN: strong isospin (I, I₃)
 *   PARITY: matter/antimatter
 */

#include <stdint.h"
#include <stdbool.h"
#include <math.h"
#include <stdio.h>

/*=== ENUMERATIONS ===*/
typedef enum { UP=0, DOWN=1, CHARM=2, STRANGE=3, TOP=4, BOTTOM=5 } Flavor;
typedef enum { RED=0, GREEN=1, BLUE=2 } Color;
typedef enum { MATTER=0, ANTIMATTER=1 } Parity;

/*=== PHYSICAL CONSTANTS ===*/
/* Masses in MeV/c² (PDG 2024) */
static const double MASS_MEV[6] = {
    2.16,      /* up    (MSbar, 2GeV) */
    4.67,      /* down  (MSbar, 2GeV) */
    1270.0,    /* charm (MSbar, m_c)  */
    93.5,      /* strange (MSbar, 2GeV) */
    172570.0,  /* top   (pole mass)   */
    4180.0     /* bottom (MSbar, m_b) */
};

/* Electric charge (e) */
static const double CHARGE_E[6] = {
     2.0/3.0,  /* up     */
    -1.0/3.0,  /* down   */
     2.0/3.0,  /* charm  */
    -1.0/3.0,  /* strange */
     2.0/3.0,  /* top    */
    -1.0/3.0   /* bottom */
};

/* Spin (ℏ) */
static const double SPIN_HBAR[6] = {
    0.5, 0.5, 0.5, 0.5, 0.5, 0.5  /* all quarks are fermions */
};

/* Baryon number (1/3 per quark) */
static const double BARYON_NUM[6] = {
    1.0/3.0, 1.0/3.0, 1.0/3.0, 
    1.0/3.0, 1.0/3.0, 1.0/3.0
};

/* Strong isospin I */
static const double ISOSPIN_I[6] = {
    0.5, 0.5,   /* u,d: I=1/2 doublet */
    0.0, 0.0,   /* c,s: I=0 singlets  */
    0.0, 0.0    /* t,b: I=0 singlets  */
};

/* Isospin projection I₃ */
static const double ISOSPIN_I3[6] = {
     0.5,       /* up:    +1/2 */
    -0.5,       /* down:  -1/2 */
     0.0, 0.0,  /* charm, strange: 0 */
     0.0, 0.0   /* top, bottom: 0 */
};

/* Strangeness, Charm, Bottomness, Topness */
static const int STRANGENESS[6] = {0, 0, 0, -1, 0, 0};
static const int CHARM_NUM[6]    = {0, 0, 1, 0, 0, 0};
static const int BOTTOM_NUM[6]   = {0, 0, 0, 0, 0, -1};
static const int TOP_NUM[6]      = {0, 0, 0, 0, 1, 0};

/* Names */
static const char* FLAVOR_NAME[6] = {"up","down","charm","strange","top","bottom"};
static const char* COLOR_NAME[3]  = {"red","green","blue"};
static const char* PARITY_NAME[2] = {"","anti"};

/*=== BIT-PACKED QUARK ID (zero overhead) ===*/
typedef uint32_t QuarkID;
/* 
 * bit layout: [31:16]=reserved [15:12]=generation [11:9]=color 
 *             [8]=parity [7:5]=reserved [4:0]=flavor
 * Total 2^5=32 slots, using 6×3×2=36 combos
 */
#define QID(f,c,p) (((QuarkID)(p)<<8) | ((QuarkID)(c)<<5) | ((QuarkID)(f)))
#define Q_FLAVOR(id)  ((Flavor)((id) & 0x1F))
#define Q_COLOR(id)   ((Color)(((id)>>5) & 0x7))
#define Q_PARITY(id)  ((Parity)(((id)>>8) & 0x1))
#define Q_GENERATION(id) (((id)>>12) & 0xF)

/*=== COMPLETE QUARK PROPERTY STRUCTURE ===*/
typedef struct {
    double spin_hbar;      /* intrinsic spin (ℏ) */
    double charge_e;       /* electric charge (e) */
    double mass_mev;       /* invariant mass (MeV/c²) */
    double baryon_number;  /* baryon number */
    double isospin_i;      /* strong isospin I */
    double isospin_i3;     /* isospin projection I₃ */
    int    strangeness;    /* S quantum number */
    int    charm_num;      /* C quantum number */
    int    bottom_num;     /* B̃ quantum number */
    int    top_num;        /* T quantum number */
    Flavor flavor;         /* flavor enum */
    Color  color;          /* color charge */
    Parity parity;         /* matter/antimatter */
    bool   is_stable;      /* decays? */
    double lifetime_s;     /* mean lifetime (seconds) */
} QuarkProperties;

/*=== KERNEL: EXTRACT ALL PROPERTIES ===*/
static QuarkProperties quark_get_properties(QuarkID q) {
    Flavor f = Q_FLAVOR(q);
    Parity p = Q_PARITY(q);
    int sign = (p == MATTER) ? 1 : -1;
    
    return (QuarkProperties){
        .spin_hbar    = SPIN_HBAR[f],
        .charge_e     = sign * CHARGE_E[f],
        .mass_mev     = MASS_MEV[f],
        .baryon_number= sign * BARYON_NUM[f],
        .isospin_i    = ISOSPIN_I[f],
        .isospin_i3   = sign * ISOSPIN_I3[f],
        .strangeness  = sign * STRANGENESS[f],
        .charm_num    = sign * CHARM_NUM[f],
        .bottom_num   = sign * BOTTOM_NUM[f],
        .top_num      = sign * TOP_NUM[f],
        .flavor       = f,
        .color        = Q_COLOR(q),
        .parity       = p,
        .is_stable    = (f == UP || f == DOWN),
        .lifetime_s   = (f == UP || f == DOWN) ? INFINITY :
                        (f == TOP) ? 5e-25 : 1e-12
    };
}

/*=== KERNEL: OPERATIONS ===*/
static inline QuarkID quark_antiparticle(QuarkID q) { return q ^ (1u<<8); }
static inline QuarkID quark_color_swap(QuarkID q, Color c) { return (q & ~(0x7u<<5)) | ((uint32_t)c<<5); }
static inline bool    quark_is_heavy(QuarkID q)    { return Q_FLAVOR(q) >= CHARM; }
static inline int     quark_generation(QuarkID q)  { 
    Flavor f = Q_FLAVOR(q);
    return (f <= DOWN) ? 1 : (f <= STRANGE) ? 2 : 3;
}
static inline bool    quark_is_type(QuarkID q)     { return (q >> 9) == 0; } /* type=quark if composite=0 */

/*=== 12 CANONICAL QUARKS (spanning all properties) ===*/
static const QuarkID QUARK_TABLE[12] = {
    QID(UP,      RED,    MATTER),      /* 0:  u_r   */
    QID(UP,      GREEN,  MATTER),      /* 1:  u_g   */
    QID(UP,      BLUE,   MATTER),      /* 2:  u_b   */
    QID(DOWN,    RED,    MATTER),      /* 3:  d_r   */
    QID(DOWN,    GREEN,  MATTER),      /* 4:  d_g   */
    QID(DOWN,    BLUE,   MATTER),      /* 5:  d_b   */
    QID(CHARM,   RED,    MATTER),      /* 6:  c_r   */
    QID(STRANGE, GREEN,  MATTER),      /* 7:  s_g   */
    QID(TOP,     RED,    MATTER),      /* 8:  t_r   */
    QID(BOTTOM,  BLUE,   MATTER),      /* 9:  b_b   */
    QID(UP,      RED,    ANTIMATTER),  /*10:  ū_r   */
    QID(DOWN,    GREEN,  ANTIMATTER)   /*11:  d̄_g   */
};

/*=== DISPLAY ===*/
int main(void) {
    printf("%-3s %-18s %9s %8s %8s %8s %5s %5s %3s %3s %3s %3s %s\n",
           "#","NAME","MASS(MeV)","Q(e)","SPIN(ℏ)","BARYON","I","I₃","S","C","B̃","T","STABLE");
    printf("%.*s\n", 115, 
           "------------------------------------------------------------------------------------------------------------------");
    
    for (int i = 0; i < 12; i++) {
        QuarkProperties p = quark_get_properties(QUARK_TABLE[i]);
        printf("%-3d %-6s%-12s %9.2f %+8.3f %8.1f %+8.3f %5.1f %+5.1f %+3d %+3d %+3d %+3d %s\n",
               i,
               PARITY_NAME[p.parity],
               FLAVOR_NAME[p.flavor],
               p.mass_mev,
               p.charge_e,
               p.spin_hbar,
               p.baryon_number,
               p.isospin_i,
               p.isospin_i3,
               p.strangeness,
               p.charm_num,
               p.bottom_num,
               p.top_num,
               p.is_stable ? "✓" : "✗");
    }
    
    /* Demonstrate operations */
    printf("\n=== OPERATIONS ===\n");
    QuarkID u_r = QUARK_TABLE[0];
    QuarkID ubar_r = quark_antiparticle(u_r);
    printf("u_r  -> antiparticle: %s%s\n", 
           PARITY_NAME[Q_PARITY(ubar_r)], FLAVOR_NAME[Q_FLAVOR(ubar_r)]);
    
    QuarkID u_g = quark_color_swap(u_r, GREEN);
    printf("u_r  -> color swap to green: %s_%s\n",
           FLAVOR_NAME[Q_FLAVOR(u_g)], COLOR_NAME[Q_COLOR(u_g)]);
    
    printf("t_r is heavy: %s\n", quark_is_heavy(QUARK_TABLE[8]) ? "true" : "false");
    printf("u_r generation: %d\n", quark_generation(u_r));
    
    return 0;
}