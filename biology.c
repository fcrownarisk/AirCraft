#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>


// DATA STRUCTURES

typedef enum { A=0, T=1, U=2, G=3, C=4 } Nucleotide;
typedef enum { mRNA_TYPE, tRNA_TYPE, rRNA_TYPE } RNA_Type;

typedef struct {
    Nucleotide *bases;
    int length;
    bool is_double_stranded;
} DNAStrand;

typedef struct {
    Nucleotide *bases;
    int length;
    RNA_Type type;
    char *sequence;
} RNAStrand;

typedef struct {
    char codon[4];
    char amino_acid[4];
    bool is_stop;
} CodonEntry;

typedef struct {
    char anticodon[4];
    char amino_acid[4];
    bool is_charged;
} tRNA;

typedef struct {
    RNAStrand *rRNA_subunit_large;
    RNAStrand *rRNA_subunit_small;
    tRNA *bound_tRNAs[2];
    int current_position;
} Ribosome;

typedef struct {
    char *sequence;
    int length;
    int max_length;
} Protein;

// Logger
typedef struct {
    char **logs;
    int count;
    int capacity;
} Logger;

Logger *LOG = NULL;


// LOGGER (replaces printf)

Logger* create_logger() {
    Logger *l = (Logger*)malloc(sizeof(Logger));
    l->count = 0; l->capacity = 200;
    l->logs = (char**)malloc(l->capacity * sizeof(char*));
    return l;
}

void log_msg(Logger *l, const char *msg) {
    if (l->count >= l->capacity) {
        l->capacity *= 2;
        l->logs = (char**)realloc(l->logs, l->capacity * sizeof(char*));
    }
    l->logs[l->count] = (char*)malloc((strlen(msg)+1) * sizeof(char));
    strcpy(l->logs[l->count], msg);
    l->count++;
}

void log_fmt(Logger *l, const char *fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log_msg(l, buf);
}

void print_logs(Logger *l) {
    for (int i = 0; i < l->count; i++) puts(l->logs[i]);
}

void free_logger(Logger *l) {
    for (int i = 0; i < l->count; i++) free(l->logs[i]);
    free(l->logs); free(l);
}


// HELPERS

char nuc_to_char(Nucleotide n) {
    switch(n) {
        case A: return 'A'; case T: return 'T';
        case U: return 'U'; case G: return 'G';
        case C: return 'C'; default: return '?';
    }
}

Nucleotide char_to_nuc(char c) {
    switch(c) {
        case 'A': case 'a': return A;
        case 'T': case 't': return T;
        case 'U': case 'u': return U;
        case 'G': case 'g': return G;
        case 'C': case 'c': return C;
        default: return A;
    }
}

Nucleotide complement(Nucleotide n, bool is_rna) {
    switch(n) {
        case A: return is_rna ? U : T;
        case T: return A;
        case U: return A;
        case G: return C;
        case C: return G;
        default: return A;
    }
}

DNAStrand* create_dna(const char *seq) {
    DNAStrand *d = (DNAStrand*)malloc(sizeof(DNAStrand));
    d->length = strlen(seq);
    d->bases = (Nucleotide*)malloc(d->length * sizeof(Nucleotide));
    d->is_double_stranded = false;
    for (int i = 0; i < d->length; i++) d->bases[i] = char_to_nuc(seq[i]);
    return d;
}

void dna_to_str(DNAStrand *d, char *buf) {
    for (int i = 0; i < d->length; i++) buf[i] = nuc_to_char(d->bases[i]);
    buf[d->length] = '\0';
}

void log_dna(Logger *l, DNAStrand *d, const char *label) {
    char buf[1024], seq[1024];
    dna_to_str(d, seq);
    sprintf(buf, "%s: %s", label, seq);
    log_msg(l, buf);
}

void log_rna(Logger *l, RNAStrand *r, const char *label) {
    char buf[1024];
    sprintf(buf, "%s: %s", label, r->sequence);
    log_msg(l, buf);
}


// DNA REPLICATION

DNAStrand* dna_replicate(DNAStrand *template_strand, bool create_complement) {
    DNAStrand *new = (DNAStrand*)malloc(sizeof(DNAStrand));
    new->length = template_strand->length;
    new->bases = (Nucleotide*)malloc(new->length * sizeof(Nucleotide));
    new->is_double_stranded = template_strand->is_double_stranded;
    for (int i = 0; i < template_strand->length; i++) {
        new->bases[i] = create_complement ? 
            complement(template_strand->bases[i], false) : 
            template_strand->bases[i];
    }
    return new;
}

typedef struct { DNAStrand *strand1; DNAStrand *strand2; } ReplicationFork;

ReplicationFork* replicate_dna_full(DNAStrand *parent) {
    ReplicationFork *f = (ReplicationFork*)malloc(sizeof(ReplicationFork));
    f->strand1 = dna_replicate(parent, false);
    f->strand2 = dna_replicate(parent, true);
    return f;
}


// TRANSCRIPTION

RNAStrand* transcribe_mRNA(DNAStrand *template_dna, int start, int end) {
    if (end == -1) end = template_dna->length;
    RNAStrand *rna = (RNAStrand*)malloc(sizeof(RNAStrand));
    rna->length = end - start;
    rna->bases = (Nucleotide*)malloc(rna->length * sizeof(Nucleotide));
    rna->type = mRNA_TYPE;
    rna->sequence = (char*)malloc((rna->length + 1) * sizeof(char));
    for (int i = start; i < end; i++) {
        rna->bases[i - start] = complement(template_dna->bases[i], true);
        rna->sequence[i - start] = nuc_to_char(rna->bases[i - start]);
    }
    rna->sequence[rna->length] = '\0';
    return rna;
}

int find_promoter(DNAStrand *dna, const char *promoter_seq) {
    int plen = strlen(promoter_seq);
    for (int i = 0; i <= dna->length - plen; i++) {
        bool match = true;
        for (int j = 0; j < plen; j++) {
            if (nuc_to_char(dna->bases[i + j]) != promoter_seq[j]) {
                match = false; break;
            }
        }
        if (match) return i;
    }
    return -1;
}

RNAStrand* transcribe_gene(DNAStrand *dna, const char *promoter, const char *terminator) {
    int start = find_promoter(dna, promoter);
    if (start == -1) return NULL;
    int end = find_promoter(dna, terminator);
    if (end == -1) end = dna->length;
    return transcribe_mRNA(dna, start + strlen(promoter), end);
}

RNAStrand* transcribe_tRNA(DNAStrand *d, int s, int e) {
    RNAStrand *r = transcribe_mRNA(d, s, e);
    r->type = tRNA_TYPE;
    log_msg(LOG, "  tRNA transcribed (cloverleaf)");
    return r;
}

RNAStrand* transcribe_rRNA(DNAStrand *d, int s, int e) {
    RNAStrand *r = transcribe_mRNA(d, s, e);
    r->type = rRNA_TYPE;
    log_msg(LOG, "  rRNA transcribed (subunits)");
    return r;
}


// GENETIC CODE

CodonEntry genetic_code[] = {
    {"AUG","Met",0},{"UAA","Stop",1},{"UAG","Stop",1},{"UGA","Stop",1},
    {"UUU","Phe",0},{"UUC","Phe",0},{"UUA","Leu",0},{"UUG","Leu",0},
    {"UCU","Ser",0},{"UCC","Ser",0},{"UCA","Ser",0},{"UCG","Ser",0},
    {"UAU","Tyr",0},{"UAC","Tyr",0},{"UGU","Cys",0},{"UGC","Cys",0},
    {"UGG","Trp",0},{"CUU","Leu",0},{"CUC","Leu",0},{"CUA","Leu",0},
    {"CUG","Leu",0},{"CCU","Pro",0},{"CCC","Pro",0},{"CCA","Pro",0},
    {"CCG","Pro",0},{"CAU","His",0},{"CAC","His",0},{"CAA","Gln",0},
    {"CAG","Gln",0},{"CGU","Arg",0},{"CGC","Arg",0},{"CGA","Arg",0},
    {"CGG","Arg",0},{"AUU","Ile",0},{"AUC","Ile",0},{"AUA","Ile",0},
    {"ACU","Thr",0},{"ACC","Thr",0},{"ACA","Thr",0},{"ACG","Thr",0},
    {"AAU","Asn",0},{"AAC","Asn",0},{"AAA","Lys",0},{"AAG","Lys",0},
    {"AGU","Ser",0},{"AGC","Ser",0},{"AGA","Arg",0},{"AGG","Arg",0},
    {"GUU","Val",0},{"GUC","Val",0},{"GUA","Val",0},{"GUG","Val",0},
    {"GCU","Ala",0},{"GCC","Ala",0},{"GCA","Ala",0},{"GCG","Ala",0},
    {"GAU","Asp",0},{"GAC","Asp",0},{"GAA","Glu",0},{"GAG","Glu",0},
    {"GGU","Gly",0},{"GGC","Gly",0},{"GGA","Gly",0},{"GGG","Gly",0}
};

char* codon_to_aa(const char *codon, bool *is_stop) {
    int size = sizeof(genetic_code) / sizeof(CodonEntry);
    for (int i = 0; i < size; i++) {
        if (strncmp(genetic_code[i].codon, codon, 3) == 0) {
            *is_stop = genetic_code[i].is_stop;
            return genetic_code[i].amino_acid;
        }
    }
    *is_stop = false;
    return "???";
}

char* get_anticodon(const char *codon) {
    static char anti[4];
    for (int i = 0; i < 3; i++) {
        char b = codon[i];
        if (b == 'A') anti[i] = 'U';
        else if (b == 'U') anti[i] = 'A';
        else if (b == 'G') anti[i] = 'C';
        else if (b == 'C') anti[i] = 'G';
    }
    anti[3] = '\0';
    return anti;
}


// RIBOSOME & TRANSLATION

Ribosome* create_ribosome(RNAStrand *large, RNAStrand *small) {
    Ribosome *r = (Ribosome*)malloc(sizeof(Ribosome));
    r->rRNA_subunit_large = large;
    r->rRNA_subunit_small = small;
    r->bound_tRNAs[0] = r->bound_tRNAs[1] = NULL;
    r->current_position = 0;
    return r;
}

tRNA* create_tRNA(const char *anti, const char *aa) {
    tRNA *t = (tRNA*)malloc(sizeof(tRNA));
    strcpy(t->anticodon, anti);
    strcpy(t->amino_acid, aa);
    t->is_charged = true;
    return t;
}

Protein* translate_mRNA(RNAStrand *mrna, Ribosome *ribosome) {
    Protein *protein = (Protein*)malloc(sizeof(Protein));
    protein->max_length = mrna->length / 3 + 1;
    protein->sequence = (char*)malloc(protein->max_length * sizeof(char));
    protein->length = 0;
    
    log_msg(LOG, "\n=== TRANSLATION ===");
    log_fmt(LOG, "mRNA: %s", mrna->sequence);
    log_msg(LOG, "Ribosome assembled (rRNA + proteins)\n");
    
    int start_pos = -1;
    for (int i = 0; i <= mrna->length - 3; i++) {
        char codon[4];
        strncpy(codon, &mrna->sequence[i], 3);
        codon[3] = '\0';
        if (strcmp(codon, "AUG") == 0) { start_pos = i; break; }
    }
    
    if (start_pos == -1) {
        log_msg(LOG, "No start codon found!");
        return protein;
    }
    
    log_fmt(LOG, "Start codon (AUG) at position %d", start_pos);
    log_msg(LOG, "Initiation: Met-tRNA binds to P-site\n");
    
    int current_pos = start_pos;
    bool stop = false;
    int codon_count = 0;
    
    while (current_pos + 3 <= mrna->length && !stop) {
        char codon[4];
        strncpy(codon, &mrna->sequence[current_pos], 3);
        codon[3] = '\0';
        
        bool is_stop;
        char *aa = codon_to_aa(codon, &is_stop);
        
        if (is_stop) {
            log_fmt(LOG, "Stop codon (%s) reached", codon);
            log_msg(LOG, "Release factor binds, polypeptide released");
            stop = true;
            break;
        }
        
        char *anti = get_anticodon(codon);
        tRNA *trna = create_tRNA(anti, aa);
        log_fmt(LOG, "Codon %d: %s → %s (tRNA: %s)", codon_count+1, codon, aa, anti);
        
        if (protein->length < protein->max_length - 1) {
            if (protein->length == 0)
                strcpy(&protein->sequence[protein->length], "Met");
            else
                strcpy(&protein->sequence[protein->length], aa);
            protein->length++;
            if (protein->length < protein->max_length - 1) {
                protein->sequence[protein->length] = '-';
                protein->length++;
            }
        }
        
        log_msg(LOG, "  Peptide bond formed");
        current_pos += 3;
        codon_count++;
        ribosome->current_position = current_pos;
        free(trna);
    }
    
    protein->sequence[protein->length] = '\0';
    log_msg(LOG, "\n=== TRANSLATION COMPLETE ===");
    log_fmt(LOG, "Protein: %s", protein->sequence);
    log_fmt(LOG, "Length: %d amino acids\n", (protein->length + 1) / 2);
    
    return protein;
}


// CENTRAL DOGMA

void central_dogma_simulation(DNAStrand *gene) {
    log_msg(LOG, "\n╔════════════════════════════════════════╗");
    log_msg(LOG, "║  CENTRAL DOGMA: DNA → RNA → Protein   ║");
    log_msg(LOG, "╚════════════════════════════════════════╝\n");
    
    log_msg(LOG, "STEP 1: DNA REPLICATION");
    log_msg(LOG, "─────────────────────");
    log_dna(LOG, gene, "Original DNA");
    ReplicationFork *fork = replicate_dna_full(gene);
    log_dna(LOG, fork->strand1, "Strand 1");
    log_dna(LOG, fork->strand2, "Strand 2");
    log_msg(LOG, "✓ DNA replicated (semi-conservative)\n");
    
    log_msg(LOG, "STEP 2: TRANSCRIPTION");
    log_msg(LOG, "────────────────────");
    RNAStrand *mrna = transcribe_gene(gene, "TATA", "TTTT");
    if (mrna) {
        log_rna(LOG, mrna, "mRNA");
        log_msg(LOG, "✓ mRNA synthesized (5' cap, poly-A tail)\n");
        log_msg(LOG, "Other RNA transcripts:");
        DNAStrand *td = create_dna("TGGTTCGA");
        RNAStrand *tr = transcribe_tRNA(td, 0, -1);
        free(td->bases); free(td); free(tr->bases); free(tr->sequence); free(tr);
        DNAStrand *rd = create_dna("GGUGGCUCG");
        RNAStrand *rr = transcribe_rRNA(rd, 0, -1);
        free(rd->bases); free(rd); free(rr->bases); free(rr->sequence); free(rr);
    } else {
        log_msg(LOG, "✗ Transcription failed");
        return;
    }
    
    log_msg(LOG, "STEP 3: TRANSLATION");
    log_msg(LOG, "──────────────────");
    RNAStrand *large = transcribe_rRNA(create_dna("GGUGGCUCG"), 0, -1);
    RNAStrand *small = transcribe_rRNA(create_dna("GGUGGCUCG"), 0, -1);
    Ribosome *ribosome = create_ribosome(large, small);
    Protein *protein = translate_mRNA(mrna, ribosome);
    log_msg(LOG, "✓ Protein folded into functional 3D structure");
    
    free(gene->bases); free(gene);
    free(fork->strand1->bases); free(fork->strand1);
    free(fork->strand2->bases); free(fork->strand2);
    free(fork);
    free(mrna->bases); free(mrna->sequence); free(mrna);
    free(large->bases); free(large->sequence); free(large);
    free(small->bases); free(small->sequence); free(small);
    free(ribosome);
    free(protein->sequence); free(protein);
}


// MAIN

int main() {
    LOG = create_logger();
    log_msg(LOG, "╔════════════════════════════════════════╗");
    log_msg(LOG, "║  DNA → RNA → PROTEIN SIMULATION      ║");
    log_msg(LOG, "║  Replication • Transcription • Translation ║");
    log_msg(LOG, "╚════════════════════════════════════════╝");
    
    DNAStrand *gene = create_dna("TATATACGTATGGCTTACGATAAATTTTT");
    gene->is_double_stranded = true;
    central_dogma_simulation(gene);
    
    log_msg(LOG, "\n╔════════════════════════════════════════╗");
    log_msg(LOG, "║  SIMULATION COMPLETE                  ║");
    log_msg(LOG, "╚════════════════════════════════════════╝");
    
    print_logs(LOG);
    free_logger(LOG);
    return 0;
}
