
typedef struct {
    uae_u32   addr;
    uae_u32   size;
} segment;

struct seglist_s {
    char     *name;
    uae_u32   addr;  
    int       num_segments;
    segment  *segments;
    
    struct seglist_s *prev, *next;
};
typedef struct seglist_s seglist;

typedef struct {
    seglist *first;
    seglist *last;
    int      num_seglists;
} seglist_pool;

extern seglist_pool segtracker_pool;

uaecptr segtracker_startup(uaecptr resaddr);
void segtracker_install(void);

void segtracker_dump(const char *match);
int segtracker_search_address(uae_u32 addr, seglist **found_sl, int *num_seg);