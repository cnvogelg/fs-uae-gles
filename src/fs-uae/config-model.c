#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stddef.h>
#include <fs/conf.h>
#include "config-model.h"

amiga_config g_fs_uae_amiga_configs[CONFIG_LAST + 1] = {};
amiga_config *g_fs_uae_config = NULL;
int g_fs_uae_amiga_config = 0;
int g_fs_uae_ntsc_mode = 0;
int g_fs_uae_amiga_model = 0;

static const char *wb_disk_1_3_3 =
        "Workbench v1.3.3 rev 34.34 (1990)(Commodore)(A500-A2000)"
        "(Disk 1 of 2)(Workbench).adf";
static const char *wb_disk_2_0_4 =
        "Workbench v2.04 rev 37.67 (1991)(Commodore)"
        "(Disk 1 of 4)(Workbench).adf";
static const char *wb_disk_2_0_5 =
        "Workbench v2.05 rev 37.71 (1992)(Commodore)"
        "(Disk 1 of 4)(Workbench).adf";
static const char *wb_disk_3_1_0 =
        "Workbench v3.1 rev 40.42 (1994)(Commodore)(M10)"
        "(Disk 2 of 6)(Workbench)[!].adf";

static void init_common(amiga_config *c, const char *id, const char *name,
                        int model, int quickstart, int quickstart_config)
{
    c->id = id;
    c->name = name;
    c->model = model;
    c->quickstart_model = quickstart;
    c->quickstart_config = quickstart_config;
    c->fast_on_accuracy_level = -999;
    c->cpu_idle = FS_CONFIG_NONE;
    c->z3realmapping = FS_CONFIG_NONE;
}

static void init_a1200(amiga_config *c, const char *id, const char *name,
                       int quickstart_config)
{
    init_common(c, id, name, MODEL_A1200, 4, quickstart_config);
    c->enhanced_audio_filter = 1;
#ifndef NEW_ACCURACY_SYSTEM
#error do not use
    c->fast_on_accuracy_level = 0;
#endif
    c->wb_disk = wb_disk_3_1_0;
}

static void init_cd32(amiga_config *c, const char *id, const char *name,
                      int quickstart_config)
{
    init_common(c, id, name, MODEL_CD32, 8, quickstart_config);
    c->enhanced_audio_filter = 1;
#ifndef NEW_ACCURACY_SYSTEM
    c->fast_on_accuracy_level = 0;
#endif
}

static void init_a3000(amiga_config *c, const char *id, const char *name,
                       int quickstart_config)
{
    init_common(c, id, name, MODEL_A3000, 5, quickstart_config);
    c->fast_on_accuracy_level = 1;
    c->no_accuracy_adjustment = 1;
    c->allow_z3_memory = 1;
    // c->enhanced_audio_filter = 1;
    c->wb_disk = wb_disk_3_1_0;
}

static void init_a4000(amiga_config *c, const char *id, const char *name,
                       int quickstart_config)
{
    init_common(c, id, name, MODEL_A4000, 6, quickstart_config);
    c->fast_on_accuracy_level = 1;
    c->no_accuracy_adjustment = 1;
    c->allow_z3_memory = 1;
    c->enhanced_audio_filter = 1;
    c->wb_disk = wb_disk_3_1_0;
}

static void init_a4000_ppc(amiga_config *c, const char *id, const char *name,
                           int quickstart_config)
{
    init_a4000(c, id, name, quickstart_config);
    c->cpu_model = "68060";
    c->cpu_idle = 0;
    c->z3realmapping = 0;
    c->accelerator = "cyberstorm-ppc";
}

void fs_uae_init_configs()
{
    amiga_config *c;

    c = g_fs_uae_amiga_configs + CONFIG_A1000;
    init_common(c, "A1000", "Amiga 1000", MODEL_A1000, 3, 0);

    c = g_fs_uae_amiga_configs + CONFIG_A500;
    init_common(c, "A500", "Amiga 500", MODEL_A500, 0, 0);
    c->wb_disk = wb_disk_1_3_3;

    c = g_fs_uae_amiga_configs + CONFIG_A500_512K;
    init_common(c, "A500/512K", "Amiga 500 (512 KB)", MODEL_A500, 0, 3);
    c->wb_disk = wb_disk_1_3_3;

    c = g_fs_uae_amiga_configs + CONFIG_A500P;
    init_common(c, "A500+", "Amiga 500+", MODEL_A500P, 1, 0);
    c->wb_disk = wb_disk_2_0_4;

    c = g_fs_uae_amiga_configs + CONFIG_A600;
    init_common(c, "A600", "Amiga 600", MODEL_A600, 2, 0);
    c->wb_disk = wb_disk_2_0_5;

    c = g_fs_uae_amiga_configs + CONFIG_CDTV;
    init_common(c, "CDTV", "Commodore CDTV", MODEL_CDTV, 9, 0);

    c = g_fs_uae_amiga_configs + CONFIG_CD32;
    init_cd32(c, "CD32", "Amiga CD32", 0);

    c = g_fs_uae_amiga_configs + CONFIG_CD32_FMV;
    init_cd32(c, "CD32/FMV", "Amiga CD32 + FMV ROM", 1);

    c = g_fs_uae_amiga_configs + CONFIG_A1200;
    init_a1200(c, "A1200", "Amiga 1200", 0);

    c = g_fs_uae_amiga_configs + CONFIG_A1200_020;
    init_a1200(c, "A1200/020", "Amiga 1200 (68020)", 0);
    c->cpu_model = "68020";
    c->cpu_32bit_addressing = 1;
    c->allow_z3_memory = 1;

    c = g_fs_uae_amiga_configs + CONFIG_A3000;
    init_a3000(c, "A3000", "Amiga 3000", 2);

    c = g_fs_uae_amiga_configs + CONFIG_A4000;
    init_a4000(c, "A4000", "Amiga 4000", 0);

    c = g_fs_uae_amiga_configs + CONFIG_A4000_040;
    init_a4000(c, "A4000/040", "Amiga 4000 (68040)", 1);

    c = g_fs_uae_amiga_configs + CONFIG_A4000_PPC;
    init_a4000_ppc(c, "A4000/PPC", "Amiga 4000 (PPC)", 1);

    c = g_fs_uae_amiga_configs + CONFIG_A4000_OS4;
    init_a4000_ppc(c, "A4000/OS4", "Amiga 4000 (PPC / OS4)", 1);
    c->default_hd_controller = "scsi_cpuboard";
    c->default_cd_controller = "scsi_cpuboard";
    c->default_graphics_card = "picasso-iv-z3";

#if 0
    c = g_amiga_configs + CONFIG_A1200_030;
    c->id = "A1200/030";
    c->model = MODEL_A1200;
    c->name = "Amiga 1200 (68030)";
    c->quickstart = "A1200,,";
    c->cpu_model = "68030";
    c->cpu_32bit_addressing = 1;
    c->allow_z3_memory = 1;
    c->z3mem_size = 64;

    c = g_amiga_configs + CONFIG_A1200_040;
    c->id = "A1200/040";
    c->model = MODEL_A1200;
    c->name = "Amiga 1200 (68040)";
    c->quickstart = "A1200,,";
    c->cpu_model = "68040";
    c->cpu_32bit_addressing = 1;
    c->allow_z3_memory = 1;
    c->z3mem_size = 64;
    c->fast = 1;
#endif

    c = g_fs_uae_amiga_configs + CONFIG_SUPER;
    c->id = "SUPER";
    // FIXME:
    c->model = MODEL_A1200;
    c->name = "Amiga (Super)";
    //c->quickstart = "A1200,,";
    c->quickstart_model = 11;
    c->fast_on_accuracy_level = 1;
    c->no_accuracy_adjustment = 1;
    //c->cpu_model = "68020";
    //c->cpu_32bit_addressing = 1;
    c->allow_z3_memory = 1;
    //c->z3mem_size = 64;
    c->warning = "SUPER is deprecated, use A4000/040 instead";

    c = g_fs_uae_amiga_configs + CONFIG_LAST;
    c->id = NULL;
};
